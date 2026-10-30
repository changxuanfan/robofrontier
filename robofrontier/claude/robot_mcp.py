"""Experimental role-bound MCP stdio server; not yet qualified for scored runs."""
import argparse
import base64
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import re
import sys
import threading
import uuid

from robofrontier.pipeline_client import client_call
from robofrontier.tool_errors import CallIdConflict, conflict_error, uncertain_error

TOOL = {
    'name': 'robot',
    'description': ('Call the robot API bound to this role. For each new move, gripper, action_batch, delegate or stop, '
                    'provide a unique stable outer call_id. For job, put the original action ID in data.call_id '
                    'and omit the outer call_id. Retry an uncertain request only with its original outer ID and identical op and data.'),
    'inputSchema': {'type': 'object', 'properties': {
        'op': {'type': 'string'}, 'data': {'type': 'object'},
        'call_id': {'type': 'string'}, 'views': {'type': 'array', 'items': {'type': 'string'}}},
        'required': ['op', 'data'], 'additionalProperties': False},
}
# Administrative methods are never made available through the policy connection.
POLICY_OPS = frozenset(('public_image', 'ready', 'robot_model', 'check_motion', 'send', 'inbox', 'wait_events',
                        'history', 'work', 'delegate', 'return_task', 'team', 'checkpoint', 'yield',
                        'observe', 'stop', 'move', 'gripper', 'action_batch', 'job', 'finish', 'pending'))
STABLE_ID_OPS = frozenset(('move', 'gripper', 'action_batch', 'delegate', 'stop'))


class RobotBridge:
    def __init__(self, run, connection, invoke=client_call):
        self.run = Path(run).resolve()
        self.connection = Path(connection).resolve()
        expected_dir = (self.run / 'private/state/connections').resolve()
        if self.connection.parent != expected_dir or not self.connection.is_file():
            raise ValueError('Role connection must be in this run')
        config = json.loads(self.connection.read_text())
        if config.get('actor') in (None, 'Meta') or self.connection.stem != config['actor']:
            raise ValueError('Expected a policy role connection')
        if config.get('run_id') != self.run.name:
            raise ValueError('Role connection belongs to a different run')
        self.invoke = invoke

    def call(self, args):
        if not isinstance(args, dict) or set(args) - {'op', 'data', 'call_id', 'views'}:
            raise ValueError('Invalid robot arguments')
        op, data = args.get('op'), args.get('data')
        views = args.get('views', [])
        if op not in POLICY_OPS or not isinstance(data, dict):
            raise ValueError('Unknown or administrative robot operation')
        if not isinstance(views, list) or not all(isinstance(v, str) and v for v in views):
            raise ValueError('views must be an array of camera names')
        if op == 'public_image':
            return self.public_images(data)
        call_id = args.get('call_id')
        if op in STABLE_ID_OPS and (not isinstance(call_id, str) or not call_id):
            raise ValueError('Provide a stable call_id for this operation')
        call_id = call_id or uuid.uuid4().hex
        if op in ('move', 'gripper', 'action_batch') and 'cameras' not in data:
            data = {**data, 'cameras': views}
        try:
            result = self.invoke(self.connection, op, data, call_id, log_call_id=False)
        except CallIdConflict:
            return {'isError': True, 'content': [{'type': 'text', 'text':
                json.dumps(conflict_error(call_id), ensure_ascii=False)}]}
        except Exception:
            return {'isError': True, 'content': [{'type': 'text', 'text':
                json.dumps(uncertain_error(call_id), ensure_ascii=False)}]}
        if op == 'work' and 'delegate_id' in data:
            result = self.attach_delegate_report(result, data['delegate_id'])
        content = [{'type': 'text', 'text': json.dumps(result, ensure_ascii=False, allow_nan=False)}]
        observation = result.get('observation') or (result.get('original_result') or {}).get('observation') or {}
        for view in views:
            name = observation.get('images', {}).get(view)
            if not name:
                content.append({'type': 'text', 'text': 'Selected view unavailable: ' + view})
                continue
            try:
                image = Path(name).resolve()
                public = (self.run / 'public').resolve()
                if not image.is_relative_to(public) or not image.is_file():
                    raise ValueError('Observation image is outside this run public directory or unavailable')
                data_bytes = image.read_bytes()
                if data_bytes.startswith(b'\x89PNG\r\n\x1a\n'):
                    mime = 'image/png'
                elif data_bytes.startswith(b'\xff\xd8\xff'):
                    mime = 'image/jpeg'
                else:
                    raise ValueError('Observation is not a PNG or JPEG image')
            except (OSError, ValueError, TypeError, RuntimeError):
                # The RPC has already executed. Preserve its result and call ID
                # even when the optional image cannot safely be returned.
                content.append({'type': 'text', 'text': 'Selected view could not be loaded: ' + view})
                continue
            content.append({'type': 'image', 'data': base64.b64encode(data_bytes).decode('ascii'), 'mimeType': mime})
        return {'content': content}

    def attach_delegate_report(self, result, actor):
        """Inline only a finished child report authorized by the runtime work response."""
        if not isinstance(actor, str) or not re.fullmatch(r'Delegate_[a-f0-9]{12}', actor):
            return result
        statuses = result.get('delegate_status')
        if not isinstance(statuses, dict):
            return result
        status = statuses.get(actor, {})
        if not isinstance(status, dict) or status.get('state') != 'finished' or not status.get('result_ref'):
            return result
        if not isinstance(status.get('delegation_id'), str) or not status['delegation_id']:
            return {**result, 'delegate_report_unavailable': True}
        root = (self.run / 'public/results').resolve()
        expected = root / (actor + '.json')
        try:
            ref = Path(status['result_ref'])
            if (not root.is_relative_to((self.run/'public').resolve()) or ref.resolve() != expected
                    or not expected.is_file() or expected.stat().st_size > 128_000):
                raise ValueError('Invalid or oversized report')
            report = json.loads(expected.read_text())
            if not isinstance(report, dict) or report.get('actor') != actor or report.get('delegation_id') != status.get('delegation_id'):
                raise ValueError('Report identity mismatch')
        except (OSError, ValueError, TypeError):
            return {**result, 'delegate_report_unavailable': True}
        return {**result, 'delegate_report': report}


    def public_images(self, data):
        """Return only reference images explicitly listed in the frozen public task."""
        if set(data) != {'names'} or not isinstance(data['names'], list) or not data['names']:
            raise ValueError('Provide names from the public task initial_reference_images')
        if len(data['names']) > 8 or len(set(data['names'])) != len(data['names']):
            raise ValueError('Select at most eight distinct public reference images')
        root = (self.run / 'public/inputs').resolve()
        task = json.loads((root / 'task.json').read_text())
        allowed = set(task.get('initial_reference_images', []))
        if task.get('robot_layout_image'):
            allowed.add(task['robot_layout_image'])
        target = task.get('target')
        if isinstance(target, dict) and isinstance(target.get('reference_image'), str):
            allowed.add(target['reference_image'])
        content = []
        for name in data['names']:
            if not isinstance(name, str) or name not in allowed:
                raise ValueError('Image is not in the frozen public task')
            # Family references are relative to inputs; the generated robot
            # layout uses the run-relative public/inputs/ prefix.
            path = ((self.run / name) if name.startswith('public/inputs/') else (root / name)).resolve()
            if not path.is_relative_to(root) or not path.is_file():
                raise ValueError('Public reference image is unavailable')
            data_bytes = path.read_bytes()
            if not data_bytes.startswith(b'\x89PNG\r\n\x1a\n'):
                raise ValueError('Public reference is not a PNG image')
            content.append({'type': 'text', 'text': 'Public reference: ' + name})
            content.append({'type': 'image', 'data': base64.b64encode(data_bytes).decode('ascii'), 'mimeType': 'image/png'})
        return {'content': content}


def handle(request, bridge):
    method = request.get('method')
    params = request.get('params') or {}
    if method == 'initialize':
        return {'protocolVersion': params.get('protocolVersion', '2025-03-26'),
                'capabilities': {'tools': {}}, 'serverInfo': {'name': 'robofrontier-robot', 'version': '0.1.0-dev'}}
    if method == 'ping': return {}
    if method == 'tools/list': return {'tools': [TOOL]}
    if method == 'tools/call':
        if params.get('name') != 'robot': raise ValueError('Unknown tool')
        return bridge.call(params.get('arguments'))
    raise ValueError('Unknown MCP method')


def serve(bridge):
    lock = threading.Lock()
    def reply(request):
        if 'id' not in request: return
        try:
            response = {'jsonrpc': '2.0', 'id': request['id'], 'result': handle(request, bridge)}
        except Exception as error:
            response = {'jsonrpc': '2.0', 'id': request['id'], 'error': {'code': -32603, 'message': str(error)}}
        with lock:
            print(json.dumps(response, ensure_ascii=False), flush=True)
    with ThreadPoolExecutor(max_workers=8) as pool:
        for line in sys.stdin:
            try:
                request = json.loads(line)
                if not isinstance(request, dict): continue
                pool.submit(reply, request)
            except json.JSONDecodeError:
                with lock:
                    print(json.dumps({'jsonrpc': '2.0', 'id': None,
                                      'error': {'code': -32700, 'message': 'Parse error'}}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--connection', type=Path, required=True)
    args = parser.parse_args()
    serve(RobotBridge(args.run_dir, args.connection))


if __name__ == '__main__': main()

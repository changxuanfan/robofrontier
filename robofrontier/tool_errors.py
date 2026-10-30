"""Policy-safe tool failures. Rejection of a new request says nothing about the old one."""


class CallIdConflict(ValueError):
    def __init__(self, call_id):
        super().__init__('Call ID reused with different operation/parameters')
        self.call_id = call_id


def conflict_error(call_id):
    return {
        'tool_error_schema': 1, 'call_id': call_id, 'error_code': 'CALL_ID_CONFLICT',
        'error': 'This call ID is already associated with a different request.',
        'request_status': 'rejected', 'original_call_status': 'unknown',
        'note': 'The conflicting request was rejected. Inspect pending and, for your physical actions, job with the original ID. Use a new ID only for an intentionally new request after reconciling existing work; never replay uncertain motion.',
    }


def is_conflict_error(value, call_id):
    return (isinstance(value, dict) and type(value.get('tool_error_schema')) is int
            and value['tool_error_schema'] == 1 and value.get('call_id') == call_id
            and value.get('error_code') == 'CALL_ID_CONFLICT'
            and value.get('request_status') == 'rejected'
            and value.get('original_call_status') == 'unknown')


def uncertain_error(call_id):
    return {
        'tool_error_schema': 1, 'call_id': call_id, 'error_code': 'TRANSPORT_UNCERTAIN',
        'error': 'The tool result could not be confirmed; execution must not be inferred.',
        'request_status': 'unknown', 'original_call_status': 'unknown',
        'note': 'Inspect pending and, for your physical actions, job with the original ID. Do not create a new ID to replay an uncertain operation.',
    }

"""Current Relay task contract, independent of historical scene revisions.

The geometry stays in the existing frozen workstation folders. This module
defines current instructions and Goal acceptance once; old recorded tasks do
not acquire these semantics merely because they are read by newer code.
"""
from copy import deepcopy
from pathlib import Path
import json
import os
import hashlib

RULE_REVISION = 'relay-final-goal-50mm-v1'
BOUNDARY_TOLERANCE_M = .050
LOW_RELEASE_GAP_M = .030
CATALOG_RELATIVE = 'configs/suites/frontier-relay.json'
HARNESS_RELATIVE = 'configs/harness-frontier-v2-servo2.json'


def is_current(task):
    return task.get('completion', {}).get('rule_revision') == RULE_REVISION


def final_task(task):
    if task.get('family') != 'relay':
        raise ValueError('Relay contract requires a Relay task')
    result = deepcopy(task)
    level = result['level']
    mechanism = result['mechanism']
    count = len(mechanism['payload_assemblies'])
    rules = [
        f'Move all {count} numbered payload assemblies from their individual initial positions toward the common GOAL. START marks the route origin; returning scattered payloads to START is not required.',
        'Route regions and arrows indicate progress. Grasp points, handover locations, robot teams, allocation and execution order are planner choices. Independent work may run concurrently.',
        'Airborne transport requires at least two actual beam grasps. Establish receiving support before releasing a sender. Every assembly must undergo a real qualified airborne handover; the receiving team may retain an existing holder.',
        'Grasp the red timber, not a pendant, ring or shaft. Keep the whole assembly clear of table and floor during airborne transport, except for the allowed final GOAL landing.',
        'Any final placement in GOAL is allowed, including cargo contact and stacking. Oracle packing centers and contact-free separation are not task requirements.',
        'The complete assembly must be within the GOAL outline with at most 50mm outward geometric tolerance on each boundary. Final completion requires actual release from every robot and actual table support, directly or through other accepted cargo. There is no additional velocity threshold or stationary-duration requirement.',
        'Inside GOAL, original qualified holders may lower different parts onto the table separately. They may open before contact when the lowest assembly part is at most 30mm above the table. Opening alone is not completion: verify actual final landing, release and containment.',
        'Drops and violations remain recorded. Only the last established sending team may recover a dropped assembly; a receiver cannot pick it up to bypass an airborne handover. No reset, respawn or history erasure.',
        'Only ordinary robot actuators and real contacts may change the live task. No hand welds, object-state writes, hidden-state tools, runtime physics changes or stabilizing attachments.',
    ]
    if level > 1:
        rules.append(f'The initial endpoint chord is {mechanism["chord_m"]:.2f}m. The underside fixed eye is 100mm along the timber from its end and directly interlinks with a free eye; the wooden pendant shaft is 170mm long and carries a 600g weight. There is no mounting stem, cable or tray.')
    if level == 2:
        rules.append('The rigid circular-arc timber subtends 63 degrees.')
    if level == 3:
        rules.append('The manufactured elbow has 31.5 degrees deflection (148.5 degrees interior angle). All hinges are passive out-of-plane angular springs with ±25 degree limits, moving with the timber. With the root horizontal, the top-view centerline opening is fixed while projected lengths change.')
    result['rules'] = rules
    result['completion'] = dict(
        rule_revision=RULE_REVISION, boundary_tolerance_m=BOUNDARY_TOLERANCE_M,
        stable_duration_sim_s=0., require_stationary=False,
        allow_goal_payload_contact=True, arbitrary_goal_placement=True,
        required_airborne_handover=True, support_via_goal_payload=True,
        allow_goal_partial_support=True, allow_near_goal_release=True,
        max_release_gap_m=LOW_RELEASE_GAP_M,
        release_height_reference='lowest_assembly_part',
        scope='All assemblies released, actually grounded and inside Goal tolerance; qualified airborne handover and process history retained')
    return result


def write_task(folder):
    """Replace task metadata atomically; never edit a shared cache inode."""
    from .v2 import public_task
    folder = Path(folder)
    task = final_task(json.loads((folder / 'task.json').read_text()))
    for name, value in [('task.json', task), ('public-task.json', public_task(task))]:
        temporary = folder / (name + '.tmp')
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
        os.replace(temporary, folder / name)
    return task


def check_inputs(root, catalog):
    """Read-only validation; never creates a Trial, steps physics or runs GPT."""
    from ..workstation import Workstation
    from ..task_config import load_task
    root = Path(root)
    cases = json.loads(Path(catalog).read_text())['cases']
    if len(cases) != 9 or len({c['id'] for c in cases}) != 9 or any(c['family'] != 'relay' for c in cases):
        raise ValueError('Expected exactly nine distinct Relay inputs')
    rows = []
    for case in cases:
        folder = root / case['directory']
        source = Workstation(folder)
        task = load_task(folder / 'task.json')
        if task['id'] != case['id'] or source.manifest['case']['id'] != case['id'] or task['scene_sha256'] != source.manifest['scene_sha256']:
            raise ValueError('Relay catalog/task/source identity mismatch: ' + case['id'])
        if task != final_task(task):
            raise ValueError('Task differs from the current Relay contract: ' + case['id'])
        from .v2 import public_task
        if json.loads((folder / 'public-task.json').read_text()) != public_task(task):
            raise ValueError('Stale public Relay task: ' + case['id'])
        rows.append(dict(case_id=case['id'], scene_sha256=source.manifest['scene_sha256'],
                         task_sha256=hashlib.sha256((folder / 'task.json').read_bytes()).hexdigest()))
    return dict(rule_revision=RULE_REVISION, cases=rows, physics_stepped=False, policy_started=False)

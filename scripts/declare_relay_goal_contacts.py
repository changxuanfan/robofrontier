"""Apply the user's explicit final-contact contract without changing geometry."""
import hashlib,json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
RULE='Within GOAL, payload-to-payload contact is allowed. Any final placement inside the declared region is valid; nominal Oracle packing centers and contact-free cargo separation are not additional task requirements. Verify actual release, stability and all payloads remaining inside GOAL.'
TRANSFER_RULE='Each payload must undergo a real qualified airborne handover before the final team places and releases it inside GOAL.'
STACK_RULE='Stable stacking inside GOAL is allowed. Real tabletop support may pass through other payloads entirely inside GOAL. Every complete assembly must remain inside GOAL, released by all robots and stable; floating or robot-held loads are not complete.'

def main():
    cases=[c for c in json.loads((ROOT/'configs/suites/frontier-v2.json').read_text())['cases'] if c['family']=='relay']
    rows=[]
    for case in cases:
        folder=ROOT/case['directory'];scene=folder/'scene.xml';before=hashlib.sha256(scene.read_bytes()).hexdigest()
        path=folder/'task.json';task=json.loads(path.read_text())
        task['completion'].update(allow_goal_payload_contact=True,arbitrary_goal_placement=True,required_airborne_handover=True,support_via_goal_payload=True,rule_revision='relay-goal-contact-stack-20261026')
        if RULE not in task['rules']:task['rules'].append(RULE)
        if TRANSFER_RULE not in task['rules']:task['rules'].append(TRANSFER_RULE)
        if STACK_RULE not in task['rules']:task['rules'].append(STACK_RULE)
        path.write_text(json.dumps(task,ensure_ascii=False,indent=2)+'\n')
        assert hashlib.sha256(scene.read_bytes()).hexdigest()==before
        rows.append(dict(case_id=case['id'],scene_unchanged=True,scene_sha256=before))
    print(json.dumps(rows,indent=2))

if __name__=='__main__':main()

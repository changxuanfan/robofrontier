import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

from scripts.execute_mosaic_remaining_review import run


class CorrectionParkingFrontierTests(unittest.TestCase):
    def exercise(self,parking_fails):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);review=root/'review';(review/'requests').mkdir(parents=True)
            task=dict(scene_sha256='scene',level=3,goals=[dict(goal_id='g1',position_m=[0,0,.83]),dict(goal_id='g2',position_m=[.081,0,.83])])
            source=dict(table_buffers=[],jobs=[dict(goal='g1',object='A_stock01',path=['A']),dict(goal='g2',object='A_stock02',path=['A'])])
            observation=dict(world_id='world',review_id=1,held={},robots={'A':dict(grasp_force_setpoint_n=None)},
                evaluation=dict(violations=[],assignments={},matched_goals=0,task_success=False),
                objects={'A_stock01':dict(position_m=[.006,0,.83],rotation=[[1,0,0],[0,1,0],[0,0,1]])})
            for filename,value in [('task.json',task),('proposal.json',source),('runner-status.json',dict(phase='review',world_id='world',review_id=1)),('review/observation.json',observation)]:
                (root/filename).write_text(json.dumps(value))
            calls=[];parked=False
            def finished(code=0,error=''):
                (review/'observation.json').write_text(json.dumps(observation))
                return SimpleNamespace(returncode=code,stdout='',stderr=error)
            def physical(command,**kwargs):
                nonlocal parked
                script=next(Path(v).name for v in command if v.endswith('.py'))
                if script=='execute_mosaic_repair_transfer.py':
                    goal=command[command.index('--goal')+1];calls.append('place-'+goal)
                    if goal=='g1':return finished(1,'Original evaluator did not establish requested goal')
                    self.assertTrue(parked,'Next physical job started while the corrected arm still occupied its goal')
                    observation['evaluation'].update(assignments={'g1':'A_stock01','g2':'A_stock02'},matched_goals=2,task_success=True)
                    return finished()
                if script=='execute_mosaic_flat_pose_correction.py':
                    calls.append('correct-g1');observation['evaluation'].update(assignments={'g1':'A_stock01'},matched_goals=1)
                    return finished()
                if script=='park_mosaic_review_robot.py':
                    calls.append('park-g1')
                    if parking_fails:return finished(1,'Blocked empty-arm parking route')
                    parked=True;return finished()
                raise AssertionError(script)
            option=dict(contact_height=.035,contact_offset_m=(0.,0.,0.),centering_contact_m=0.)
            args=SimpleNamespace(trial=root,supply_plan=root/'proposal.json',id='batch',expanded_pickups=False,resume_at_full=False)
            with patch('scripts.execute_mosaic_remaining_review.subprocess.run',side_effect=physical), \
                    patch('scripts.supervise_mosaic_dropped_payloads.plan_near_goal_pick',return_value=option):
                if parking_fails:
                    with self.assertRaisesRegex(RuntimeError,'Correction parking needs owner review'):run(args)
                else:run(args)
            return calls

    def test_correction_returns_empty_actor_to_park_before_next_job(self):
        self.assertEqual(self.exercise(False),['place-g1','correct-g1','park-g1','place-g2'])

    def test_failed_parking_stops_before_next_physical_job(self):
        self.assertEqual(self.exercise(True),['place-g1','correct-g1','park-g1'])


if __name__=='__main__':unittest.main()

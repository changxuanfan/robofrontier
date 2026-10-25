"""Keep semantic transitions without repeating pose arrays or private jitter."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from robofrontier.suite.oracle_process_log import OracleProcessLog

class OracleProcessLogTests(unittest.TestCase):
    def test_acceptance_and_mutated_handover_history_survive_pose_jitter(self):
        with TemporaryDirectory() as directory:
            p=Path(directory)/'process.jsonl';log=OracleProcessLog(p)
            transitions=[]
            for i in range(50):
                state=dict(task_success=False,matched_goals=0,assignments={},private_angles_deg=[i*.01],handovers=transitions)
                if i==20:transitions.append(dict(time_s=.4,senders=['A'],receivers=['B']))
                log.record(i*.02,state,[])
            log.record(1.,dict(task_success=False,matched_goals=1,assignments={'g1':'cube'},handovers=transitions),[])
            log.record(1.02,dict(task_success=False,matched_goals=1,assignments={'g1':'cube'},handovers=transitions),[dict(kind='inter_robot_contact',sim_time_s=1.02)])
            log.close();rows=[json.loads(x) for x in p.read_text().splitlines()]
            self.assertEqual(rows[0]['schema'],OracleProcessLog.schema)
            self.assertEqual(len(rows),5)
            self.assertEqual(rows[2]['result']['handovers'][0]['receivers'],['B'])
            self.assertEqual(rows[3]['result']['assignments'],{'g1':'cube'})
            self.assertEqual(rows[-1]['result']['violations'][0]['kind'],'inter_robot_contact')
            self.assertNotIn('private_angles_deg',p.read_text())
    def test_unchanged_state_still_has_bounded_simulation_heartbeat(self):
        with TemporaryDirectory() as directory:
            p=Path(directory)/'process.jsonl';log=OracleProcessLog(p);state=dict(matched_goals=0)
            for i in range(151):log.record(i*.02,state,[])
            log.close();rows=[json.loads(x) for x in p.read_text().splitlines()][1:]
            self.assertEqual([r['sim_time_s'] for r in rows],[0.,1.,2.,3.])

if __name__=='__main__':unittest.main()

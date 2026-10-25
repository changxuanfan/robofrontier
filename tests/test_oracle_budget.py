from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from robofrontier.suite.oracle_budget import ExplicitPauseBudget


class PauseBudgetTests(unittest.TestCase):
    def test_only_acknowledged_pause_is_excluded(self):
        with TemporaryDirectory() as directory:
            now=[100.];budget=ExplicitPauseBudget(Path(directory),clock=lambda:now[0])
            now[0]+=10;budget.update(dict(paused=True,reason='User compaction'))
            now[0]+=300;self.assertEqual(budget.times()['active_wall_s'],10)
            budget.update(dict(paused=False));now[0]+=20
            self.assertEqual(budget.times(),dict(total_wall_s=330,paused_wall_s=300,active_wall_s=30))
            # A process stall or external SIGSTOP without an acknowledged
            # explicit pause still counts; no wall-gap guessing.
            now[0]+=500;self.assertEqual(budget.times()['active_wall_s'],530)

    def test_duplicate_pause_does_not_reset_start(self):
        with TemporaryDirectory() as directory:
            now=[0.];budget=ExplicitPauseBudget(Path(directory),clock=lambda:now[0])
            now[0]=3;budget.update(dict(paused=True));now[0]=8
            budget.update(dict(paused=True));now[0]=13;budget.update(dict(paused=False))
            self.assertEqual(budget.times()['paused_wall_s'],10)
            self.assertEqual(len((Path(directory)/'pause-events.jsonl').read_text().splitlines()),2)


if __name__=='__main__':unittest.main()

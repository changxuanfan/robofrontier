import unittest
from robofrontier.suite.mosaic_concurrency import dependencies,ready_scopes


class ConcurrencyPlanningTests(unittest.TestCase):
    def job(self,robot,lo,hi):return dict(robots=[robot],bounds_m=[lo,hi])

    def test_four_or_more_ready_scopes_have_no_artificial_pair_limit(self):
        jobs=[self.job(str(i),[i*2,0,0],[i*2+1,1,1]) for i in range(8)]
        pred=dependencies(jobs)
        self.assertEqual(ready_scopes(pred,set()),list(range(8)))

    def test_shared_robot_and_payload_space_preserve_source_dependencies(self):
        jobs=[self.job('A',[0,0,0],[1,1,1]),self.job('B',[2,0,0],[3,1,1]),
              self.job('C',[.5,0,0],[1.5,1,1]),self.job('A',[4,0,0],[5,1,1])]
        pred=dependencies(jobs)
        self.assertEqual(ready_scopes(pred,set()),[0,1])
        self.assertEqual(ready_scopes(pred,{0},active={1}),[2,3])

    def test_clearance_reserve_is_retained(self):
        jobs=[self.job('A',[0,0,0],[1,1,1]),self.job('B',[1.03,0,0],[2,1,1])]
        self.assertEqual(dependencies(jobs),[[],[0]])


if __name__=='__main__':unittest.main()

import unittest
import numpy as np
from robofrontier.suite.joint_route import connect_route

class JointRouteTests(unittest.TestCase):
    def test_connects_around_a_barrier_and_keeps_every_shortcut_checked(self):
        start=np.array([-.8,-.8]);goal=np.array([.8,-.8]);limits=np.array([[-1,1],[-1,1.]])
        def edge(a,b):
            samples=a+(b-a)*np.linspace(0,1,101)[:,None]
            return not np.any((abs(samples[:,0])<.12)&(samples[:,1]<.65))
        self.assertFalse(edge(start,goal));path=connect_route(start,goal,limits,edge,np.random.default_rng(7))
        self.assertIsNotNone(path);np.testing.assert_allclose(path[0],start);np.testing.assert_allclose(path[-1],goal)
        for a,b in zip(path,path[1:]):self.assertTrue(edge(a,b))
    def test_blocked_search_is_finite(self):
        result=connect_route(np.zeros(2),np.ones(2),np.array([[-1,1],[-1,1]]),lambda *_:False,np.random.default_rng(1),iterations=20)
        self.assertIsNone(result)

if __name__=='__main__':unittest.main()

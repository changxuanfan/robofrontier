"""A narrow forbidden interval between coarse samples must be caught by refinement."""
import unittest
import numpy as np
from robofrontier.suite.oracle import joint_edge_clear

class OracleRouteSamplingTests(unittest.TestCase):
    def test_finer_sampling_catches_between_sample_intersection(self):
        start=np.zeros(7);goal=np.array([.24,0,0,0,0,0,0])
        # Analogue of the observed collision at 82.5% of an admitted joint edge.
        clear=lambda q:not(.81<q[0]/.24<.84)
        self.assertTrue(joint_edge_clear(start,goal,clear))
        self.assertFalse(joint_edge_clear(start,goal,clear,.005))
    def test_invalid_resolution_rejected_and_fine_clear_edge_retained(self):
        for step in (0.,-.01,.03,float('nan')):
            with self.assertRaises(ValueError):joint_edge_clear(np.zeros(7),np.ones(7),lambda q:True,step)
        self.assertTrue(joint_edge_clear(np.zeros(7),np.ones(7),lambda q:True,.005))

if __name__=='__main__':unittest.main()

import unittest
from scripts.summarize_mosaic_parallel_phase import interval_profile,critical_path


class ParallelPhaseTests(unittest.TestCase):
    def test_real_overlap_and_adjacent_intervals_are_distinct(self):
        self.assertEqual(interval_profile([(0,2),(1,3)]),{1:2.,2:1.})
        self.assertEqual(interval_profile([(0,2),(2,4)]),{1:4.})

    def test_bound_uses_the_dependency_graph_not_the_sum_of_all_work(self):
        self.assertEqual(critical_path([2,3],[[],[]]),(3,[1]))
        self.assertEqual(critical_path([1,2,3,4],[[],[0],[0],[1,2]]),(8,[0,2,3]))


if __name__=='__main__':unittest.main()

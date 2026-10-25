from itertools import product
import unittest
import numpy as np
from scripts.simulate_dial_feedback import entropy_tree,coordinate_record


class FeedbackSimulationTests(unittest.TestCase):
    def test_record_does_not_hide_information_for_one_dial(self):
        codes=np.asarray(list(product((1,2,3),repeat=1)),dtype=np.uint8)
        a=entropy_tree(codes,'current');b=entropy_tree(codes,'record')
        np.testing.assert_array_equal(a['queries'],b['queries'])
        self.assertEqual(sorted(a['queries']),[1,2,3])

    def test_unchanged_record_is_an_inequality_not_an_exact_score(self):
        truth=np.array([1,2,3,1]);first=np.array([1,1,1,1]);next_guess=np.array([3,3,3,3])
        best=int(np.sum(first==truth));score=int(np.sum(next_guess==truth));visible=max(best,score)
        self.assertEqual((best,score,visible),(2,1,2))
        codes=np.asarray(list(product((1,2,3),repeat=4)),dtype=np.uint8)
        old=np.sum(codes==first,axis=1)==best
        new=old&(np.sum(codes==next_guess,axis=1)<=visible)
        self.assertTrue(np.any(np.all(codes[new]==truth,axis=1)))
        wrong=old&(np.sum(codes==next_guess,axis=1)==visible)
        self.assertFalse(np.any(np.all(codes[wrong]==truth,axis=1)))

    def test_coordinate_strategy_reaches_the_claimed_worst_case_bound(self):
        for n in (1,2,4):
            codes=np.asarray(list(product((1,2,3),repeat=n)),dtype=np.uint8)
            result=coordinate_record(codes)
            self.assertEqual(int(np.max(result['queries'])),1+2*n)
            self.assertEqual(len(result['queries']),3**n)

    def test_all_tree_modes_finish_every_small_password(self):
        codes=np.asarray(list(product((1,2,3),repeat=3)),dtype=np.uint8)
        for mode,neighbors in [('current',False),('record',False),('record',True)]:
            result=entropy_tree(codes,mode,incumbent_neighbors=neighbors)
            self.assertEqual(len(result['queries']),27)
            self.assertTrue(np.all(result['queries']>=1))
            self.assertTrue(np.all(result['moves']>=3))


if __name__=='__main__':unittest.main()

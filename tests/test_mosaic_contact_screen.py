import unittest
from types import SimpleNamespace
import numpy as np
from scripts.benchmark_mosaic_contact_screen import relevant_contact_indices


class ContactPrefilterTests(unittest.TestCase):
    def test_retains_self_and_carried_contacts_and_preserves_original_order(self):
        contact=SimpleNamespace(geom=np.array([[0,1],[1,2],[2,3],[4,3],[3,1],[4,5]]),dist=np.array([0.,0.,0.,0.,.003,0.]))
        scratch=SimpleNamespace(contact=contact);owners=np.array([-1,0,0,-1,-1,1])
        self.assertEqual(relevant_contact_indices(scratch,owners,0,{4},.003).tolist(),[0,1,2,3,5])

    def test_empty_and_nan_follow_the_original_skip_condition(self):
        scratch=SimpleNamespace(contact=SimpleNamespace(geom=np.empty((0,2),int),dist=np.empty(0)))
        self.assertEqual(relevant_contact_indices(scratch,np.array([-1,0]),0,set(),.003).tolist(),[])
        scratch.contact=SimpleNamespace(geom=np.array([[0,1],[0,1]]),dist=np.array([np.nan,.003]))
        self.assertEqual(relevant_contact_indices(scratch,np.array([-1,0]),0,set(),.003).tolist(),[0])

    def test_random_rows_match_scalar_relevance_with_allowed_early_rejections(self):
        rng=np.random.default_rng(1907);owners=np.array([-1,0,1,2,0,-1,1,-1])
        for count in (1,10,1000):
            pairs=rng.integers(0,len(owners),(count,2));dist=rng.uniform(-.002,.01,count)
            scratch=SimpleNamespace(contact=SimpleNamespace(geom=pairs,dist=dist))
            for robot in (0,1,2):
                for held in (set(),{5,7}):
                    expected=[i for i,(a,b) in enumerate(pairs) if not dist[i]>=.003 and (owners[a]==robot or owners[b]==robot or a in held or b in held)]
                    self.assertEqual(relevant_contact_indices(scratch,owners,robot,held,.003).tolist(),expected)


if __name__=='__main__':unittest.main()

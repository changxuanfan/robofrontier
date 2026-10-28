import unittest
from robofrontier.suite.contact_episodes import ContactEpisodes


class ContactEpisodeTests(unittest.TestCase):
    def test_continuous_chatter_and_new_contact(self):
        tracker = ContactEpisodes()
        pair = ('inter_robot_contact', 'A / B')
        for t, seen in [(0, [pair, pair]), (.01, [pair]), (.03, []), (.06, [pair]),
                        (.08, []), (.12, [pair]), (.18, []), (.20, [pair])]:
            tracker.sample(t, seen)
        report = tracker.report()
        self.assertEqual(report['inter_robot_episodes'], 3)
        self.assertEqual(report['by_pair'][0]['episodes'], 3)
        self.assertEqual(report['gap_merge_sim_s'], .05)

    def test_distinct_pairs_and_self_contacts_are_counted_separately(self):
        tracker = ContactEpisodes()
        tracker.sample(1, [('inter_robot_contact', 'A / B'), ('inter_robot_contact', 'A / C'),
                           ('self_robot_contact', 'A1 / A2')])
        tracker.sample(1.01, [])
        report = tracker.report()
        self.assertEqual(report['inter_robot_episodes'], 2)
        self.assertEqual(report['self_robot_episodes'], 1)
        self.assertAlmostEqual(report['max_sample_gap_sim_s'], .01)
        with self.assertRaises(ValueError): tracker.sample(.9, [])

    def test_sparse_samples_do_not_invent_a_separation(self):
        tracker = ContactEpisodes()
        pair = ('inter_robot_contact', 'A / B')
        tracker.sample(0, [pair])
        tracker.sample(.2, [pair])
        self.assertEqual(tracker.report()['inter_robot_episodes'], 1)
        self.assertAlmostEqual(tracker.report()['max_sample_gap_sim_s'], .2)

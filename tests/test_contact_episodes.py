import unittest
from types import SimpleNamespace
import numpy as np
from robofrontier.suite.contact_episodes import ContactEpisodes
from robofrontier.suite.evaluate import Monitor


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

    def test_physical_step_counts_thresholded_body_pairs_without_changing_violations(self):
        model = SimpleNamespace(geom_bodyid=[0, 1, 2], body=lambda i: SimpleNamespace(name=['A_link', 'B_link', 'A_other'][i]))
        monitor = SimpleNamespace(active=True, m=model, d=SimpleNamespace(time=0.),
            geom_owner=np.array([0, 1, 0]), first_collisions={}, self_contact_pairs=[],
            contact_episodes=ContactEpisodes(), next_sample=float('inf'), violations=[])
        monitor.contacts=lambda: (np.array([[0, 1], [0, 2]]), np.array([-.0003, -.0011]))
        monitor.violation=lambda kind, **details: monitor.violations.append((kind,details['key']))
        Monitor.physical_step(monitor)
        monitor.d.time=.01
        Monitor.physical_step(monitor)
        report=monitor.contact_episodes.report()
        self.assertEqual((report['inter_robot_episodes'], report['self_robot_episodes']), (1, 1))
        self.assertEqual(len(monitor.violations), 2)
        monitor.contacts=lambda: (np.empty((0,2), dtype=int), np.array([]))
        monitor.d.time=.03
        Monitor.physical_step(monitor)
        monitor.contacts=lambda: (np.array([[0, 1]]), np.array([-.0003]))
        monitor.d.time=.08
        Monitor.physical_step(monitor)
        self.assertEqual(monitor.contact_episodes.report()['inter_robot_episodes'], 2)
        self.assertEqual(len(monitor.violations), 2)

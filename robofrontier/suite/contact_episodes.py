"""Count sampled robot-contact episodes independently of scoring.

An episode is per body pair and contact kind. Contact returning after a gap
strictly longer than 50 ms of simulation time starts a new episode. Gaps up to
and including that threshold are joined to suppress solver chatter.
"""
import math


class ContactEpisodes:
    GAP_S = 0.05
    VERSION = 'robot_body_pair_contact_v1'

    def __init__(self):
        self.last_seen = {}
        self.counts = {}
        self.last_sample = None
        self.max_sample_gap_s = 0.0
        self.previous_contacts = set()

    def sample(self, time_s, contacts):
        t = float(time_s)
        if not math.isfinite(t) or (self.last_sample is not None and t < self.last_sample):
            raise ValueError('Contact samples must have finite nondecreasing simulation time')
        if self.last_sample is not None:
            self.max_sample_gap_s = max(self.max_sample_gap_s, t - self.last_sample)
        self.last_sample = t
        current = set(contacts)
        for kind, key in current:
            pair = (kind, key)
            last = self.last_seen.get(pair)
            if last is None or (pair not in self.previous_contacts and t - last > self.GAP_S + 1e-9):
                self.counts[pair] = self.counts.get(pair, 0) + 1
            self.last_seen[pair] = t
        self.previous_contacts = current

    def report(self):
        return {
            'version': self.VERSION,
            'gap_merge_sim_s': self.GAP_S,
            'max_sample_gap_sim_s': self.max_sample_gap_s,
            'scope': 'Detected robot body-pair contacts, not unique physical impacts',
            'inter_robot_episodes': sum(v for (kind, _), v in self.counts.items() if kind == 'inter_robot_contact'),
            'self_robot_episodes': sum(v for (kind, _), v in self.counts.items() if kind == 'self_robot_contact'),
            'by_pair': [dict(kind=kind, key=key, episodes=count)
                        for (kind, key), count in sorted(self.counts.items())],
        }

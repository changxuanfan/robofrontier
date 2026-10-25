import unittest
from scripts.run_relay_live_oracle import total_stop_requested

class RelayTotalDeadlineTests(unittest.TestCase):
    def test_normal_run_has_no_elapsed_time_deadline(self):
        self.assertFalse(total_stop_requested(10**9,0))
    def test_explicit_operator_stop_remains_available(self):
        self.assertTrue(total_stop_requested(1,0,True))
    def test_only_an_explicit_positive_limit_enables_a_deadline(self):
        self.assertFalse(total_stop_requested(10,20))
        self.assertTrue(total_stop_requested(21,20))

if __name__=='__main__':unittest.main()

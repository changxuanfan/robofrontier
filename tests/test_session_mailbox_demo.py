"""Concurrency and lifecycle checks for the mailbox, without making model calls."""
import concurrent.futures
import importlib.util
import json
import re
import sys
import tempfile
import time
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location("session_mailbox_demo", Path(__file__).resolve().parents[1] / "scripts/session_mailbox_demo.py")
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


class MailboxTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.pilot = module.Pilot(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def test_start_requires_both_ready(self):
        self.pilot.dispatch("ready", "A", {})
        with self.assertRaises(ValueError):
            self.pilot.dispatch("start", "driver", {})
        self.pilot.dispatch("ready", "B", {})
        self.pilot.dispatch("start", "driver", {})
        self.assertTrue(self.pilot.state["started_at"])

    def test_concurrent_sends_are_not_lost_and_retries_are_idempotent(self):
        def send(i):
            return self.pilot.dispatch("send", "A", {"key": str(i), "to": "B", "text": str(i)})
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
            answers = list(pool.map(send, range(30)))
        self.assertEqual(len(self.pilot.inbox("B")), 30)
        replay = send(4)
        self.assertTrue(replay["replayed"])
        self.assertEqual(len(self.pilot.inbox("B")), 30)
        message = answers[4]["message"]
        self.pilot.dispatch("inbox", "B", {})
        self.assertEqual(len(self.pilot.inbox("B")), 30, "Reading must not silently acknowledge")
        with self.assertRaises(ValueError):
            self.pilot.dispatch("handled", "A", {"message_id": message["id"]})
        self.pilot.dispatch("handled", "B", {"message_id": message["id"]})
        self.assertEqual(len(self.pilot.inbox("B")), 29)
        events = [json.loads(line) for line in (Path(self.temp.name) / "events.jsonl").read_text().splitlines()]
        self.assertEqual([e["seq"] for e in events], list(range(1, len(events) + 1)))

    def test_work_does_not_pause_for_messages(self):
        for agent in ("A", "B"):
            self.pilot.dispatch("ready", agent, {})
        self.pilot.dispatch("start", "driver", {})
        self.pilot.dispatch("start_job", "B", {"duration_s": 0.15})
        initial_end = self.pilot.state["job"]["ends_epoch"]
        message = self.pilot.dispatch("send", "A", {"key": "during-work", "to": "B", "text": "test"})["message"]
        self.pilot.dispatch("handled", "B", {"message_id": message["id"]})
        self.assertEqual(self.pilot.state["job"]["ends_epoch"], initial_end)
        time.sleep(0.18)
        self.assertEqual(self.pilot.snapshot()["job"]["status"], "COMPLETED")
        self.assertEqual(len([e for e in self.pilot.state["events"] if e["kind"] == "JOB_STARTED"]), 1)

    def test_standalone_report_preserves_messages_and_unread_counts(self):
        text = "hello </script> & goodbye"
        self.pilot.dispatch("send", "A", {"key": "saved", "to": "B", "text": text})
        path = module.export_dashboard(Path(self.temp.name), self.pilot.snapshot())
        html = path.read_text()
        embedded = re.search(r"window\.PILOT_SNAPSHOT=(.*?);</script>", html).group(1)
        snapshot = json.loads(embedded)
        self.assertEqual(snapshot["messages"][0]["text"], text)
        self.assertEqual(snapshot["agents"]["B"]["unread"], 1)
        self.assertNotIn("</script>", embedded)


if __name__ == "__main__":
    unittest.main()

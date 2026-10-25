#!/usr/bin/env python3
"""Historical two-thread communication pilot; not the current robot harness.

No model calls and no robot control happen in this process. The desktop parent
thread is the explicitly logged driver for creating/resuming Codex tasks.
"""
from __future__ import annotations

import argparse
import json
import threading
import time
import uuid
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
AGENTS = ("A", "B")


def stamp():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def write_json(path, data):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(path)


def export_dashboard(directory, state):
    state = json.loads(json.dumps(state))
    state["server_time"] = (state.get("verification") or {}).get("verified_at", stamp())
    if state.get("job"):
        job = state["job"]
        job["elapsed_s"] = min(job["duration_s"], max(0, time.time() - job["starts_epoch"]))
        job["progress"] = min(1, job["elapsed_s"] / job["duration_s"])
    for agent, info in state["agents"].items():
        info["unread"] = sum(m["to"] == agent and m.get("handled_at") is None for m in state["messages"])
    template = (ROOT / "assets/session_mailbox_dashboard.html").read_text()
    snapshot = json.dumps(state, ensure_ascii=False).replace("<", "\\u003c")
    template = template.replace("<script>", "<script>window.PILOT_SNAPSHOT=" + snapshot + ";</script><script>", 1)
    path = Path(directory) / "report.html"
    path.write_text(template)
    return path


class Pilot:
    def __init__(self, run_dir):
        self.directory = Path(run_dir).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.condition = threading.Condition(threading.RLock())
        self.state = {
            "run_id": self.directory.name,
            "created_at": stamp(),
            "started_at": None,
            "driver": "current Codex desktop conversation; not an autonomous daemon",
            "simulation": "none; timed dummy work only",
            "agents": {a: {"status": "NOT_STARTED", "ready": False,
                "requested_model": "gpt-6-luna", "requested_effort": "xhigh",
                "thread_id": None, "verified_model": None,
                "verified_effort": None} for a in AGENTS},
            "job": None, "messages": [], "events": [], "verification": None,
        }
        if (self.directory / "state.json").exists():
            raise ValueError("Run directory already contains a state; use a new directory")
        self._event("RUN_CREATED", "driver", {"simulation": "none"})

    def _event(self, kind, actor, details=None):
        event = {"seq": len(self.state["events"]) + 1, "timestamp": stamp(),
                 "time_ns": time.time_ns(), "kind": kind, "actor": actor,
                 "details": details or {}}
        self.state["events"].append(event)
        with (self.directory / "events.jsonl").open("a") as file:
            file.write(json.dumps(event, ensure_ascii=False) + "\n")
        write_json(self.directory / "state.json", self.state)
        # Constructor calls _event before any other thread exists.
        with self.condition:
            self.condition.notify_all()
        return event

    def _finish_due(self):
        job = self.state["job"]
        if job and job["status"] == "RUNNING" and time.time() >= job["ends_epoch"]:
            job.update(status="COMPLETED", completed_at=stamp())
            self._event("JOB_FINISHED", job["agent"], {"job_id": job["id"]})

    def inbox(self, agent):
        return [m for m in self.state["messages"]
                if m["to"] == agent and m.get("handled_at") is None]

    def snapshot(self):
        with self.condition:
            self._finish_due()
            result = json.loads(json.dumps(self.state))
            result["server_time"] = stamp()
            job = result["job"]
            if job:
                job["elapsed_s"] = min(job["duration_s"], max(0, time.time() - job["starts_epoch"]))
                job["progress"] = min(1, job["elapsed_s"] / job["duration_s"])
            for a in AGENTS:
                result["agents"][a]["unread"] = len(self.inbox(a))
            return result

    def dispatch(self, op, actor, data):
        with self.condition:
            self._finish_due()
            if actor not in (*AGENTS, "driver"):
                raise ValueError("Unknown actor")
            if op == "ready":
                if actor not in AGENTS:
                    raise ValueError("A worker must report READY")
                if not self.state["agents"][actor]["ready"]:
                    self.state["agents"][actor].update(ready=True, status="READY")
                    self._event("READY", actor)
            elif op == "bind":
                if actor != "driver":
                    raise ValueError("Only the driver binds desktop threads")
                target = data["target"]
                self.state["agents"][target].update(data["fields"])
                self._event("THREAD_BOUND", actor, {"target": target, **data["fields"]})
            elif op == "start":
                if actor != "driver" or not all(a["ready"] for a in self.state["agents"].values()):
                    raise ValueError("START requires both READY and the driver")
                if self.state["started_at"] is None:
                    self.state["started_at"] = stamp()
                    self._event("START", actor)
            elif op == "start_job":
                if actor != "B" or not self.state["started_at"]:
                    raise ValueError("Only B may start the dummy job after START")
                if self.state["job"] is None:
                    duration = float(data.get("duration_s", 150))
                    if not 0.05 <= duration <= 600:
                        raise ValueError("Invalid duration")
                    now = time.time()
                    self.state["job"] = {"id": "B-work-1", "agent": "B", "status": "RUNNING",
                        "started_at": stamp(), "starts_epoch": now,
                        "ends_epoch": now + duration, "duration_s": duration}
                    self.state["agents"][actor]["status"] = "WORKING"
                    self._event("JOB_STARTED", actor, {"duration_s": duration, "job_id": "B-work-1"})
            elif op == "send":
                recipient = data["to"]
                if recipient not in AGENTS or actor not in AGENTS:
                    raise ValueError("Messages must be between A and B")
                key = data["key"]
                existing = next((m for m in self.state["messages"]
                                 if m["from"] == actor and m["key"] == key), None)
                if existing:
                    if existing["to"] != recipient or existing["text"] != data["text"]:
                        raise ValueError("Idempotency key reused with different content")
                    return {"ok": True, "message": existing, "replayed": True}
                message = {"id": "msg-" + uuid.uuid4().hex[:12], "key": key,
                    "from": actor, "to": recipient, "text": data["text"],
                    "reply_to": data.get("reply_to"), "sent_at": stamp(),
                    "sent_epoch": time.time(), "handled_at": None}
                self.state["messages"].append(message)
                with (self.directory / "messages.jsonl").open("a") as file:
                    file.write(json.dumps(message, ensure_ascii=False) + "\n")
                self._event("MESSAGE_SENT", actor, {"message": message.copy(),
                    "job_status_at_send": self.state["job"]["status"] if self.state["job"] else None,
                    "recipient_status_at_send": self.state["agents"][recipient]["status"]})
                return {"ok": True, "message": message}
            elif op == "inbox":
                self._event("INBOX_READ", actor, {"ids": [m["id"] for m in self.inbox(actor)]})
            elif op == "handled":
                message = next((m for m in self.state["messages"] if m["id"] == data["message_id"]), None)
                if not message or message["to"] != actor:
                    raise ValueError("Only the recipient may mark a message handled")
                if message["handled_at"] is None:
                    message["handled_at"] = stamp()
                    self._event("MESSAGE_HANDLED", actor, {"message_id": message["id"],
                        "key": message["key"], "job_status": self.state["job"]["status"] if self.state["job"] else None})
            elif op == "phase":
                if actor not in AGENTS:
                    raise ValueError("Workers own their declared phases")
                self.state["agents"][actor]["status"] = data["status"]
                self._event("AGENT_PHASE", actor, data)
            elif op == "driver_event":
                if actor != "driver":
                    raise ValueError("Driver only")
                if data["kind"] == "THREAD_IDLE_CONFIRMED":
                    self.state["agents"][data["target"]]["status"] = "THREAD_IDLE_CONFIRMED"
                self._event(data["kind"], actor, {k: v for k, v in data.items() if k != "kind"})
            elif op == "watch":
                timeout = min(20, max(0, float(data.get("timeout", 10))))
                deadline = time.monotonic() + timeout
                condition = data.get("condition", "message")
                while True:
                    self._finish_due()
                    job = self.state["job"]
                    matched = {
                        "start": bool(self.state["started_at"]),
                        "job_running": bool(job and job["status"] == "RUNNING"),
                        "job_done": bool(job and job["status"] == "COMPLETED"),
                        "message": bool(self.inbox(actor)),
                        "message_or_job_done": bool(self.inbox(actor) or (job and job["status"] == "COMPLETED")),
                    }.get(condition)
                    if matched is None:
                        raise ValueError("Unknown watch condition")
                    if matched or time.monotonic() >= deadline:
                        break
                    self.condition.wait(min(0.25, max(0, deadline - time.monotonic())))
                return {"ok": True, "condition_met": matched,
                        "started": bool(self.state["started_at"]),
                        "job": self.snapshot()["job"], "inbox": self.inbox(actor)}
            elif op == "state":
                return self.snapshot()
            elif op == "verify":
                return self.verify()
            else:
                raise ValueError("Unknown operation")
            return {"ok": True, "started": bool(self.state["started_at"]),
                    "job": self.snapshot()["job"], "inbox": self.inbox(actor)}

    def verify(self):
        self._finish_due()
        events = self.state["events"]
        messages = self.state["messages"]

        def event(kind, actor=None, key=None):
            return next((e for e in events if e["kind"] == kind
                         and (actor is None or e["actor"] == actor)
                         and (key is None or e["details"].get("key") == key
                              or e["details"].get("message", {}).get("key") == key)), None)

        start = event("START")
        job_start, job_end = event("JOB_STARTED"), event("JOB_FINISHED")
        active_send = event("MESSAGE_SENT", "A", "during-work")
        active_handled = event("MESSAGE_HANDLED", "B", "during-work")
        active_reply = event("MESSAGE_SENT", "B", "during-work-ack")
        idle = next((e for e in events if e["kind"] == "THREAD_IDLE_CONFIRMED"
                     and e["details"].get("target") == "B"), None)
        idle_send = event("MESSAGE_SENT", "A", "idle-wake")
        wake = next((e for e in events if e["kind"] == "WAKE_DISPATCHED"
                     and e["details"].get("target") == "B"), None)
        idle_handled = event("MESSAGE_HANDLED", "B", "idle-wake")
        idle_reply = event("MESSAGE_SENT", "B", "idle-wake-ack")

        def ordered(*items):
            return all(items) and all(x["seq"] < y["seq"] for x, y in zip(items, items[1:]))

        checks = {
            "both_ready_before_start": bool(start and all(event("READY", a) and event("READY", a)["seq"] < start["seq"] for a in AGENTS)),
            "both_actual_luna_xhigh": all(a["verified_model"] == "gpt-6-luna" and a["verified_effort"] == "xhigh" for a in self.state["agents"].values()),
            "message_sent_while_work_running": bool(ordered(job_start, active_send, job_end)),
            "B_processed_and_replied_before_job_finished": bool(ordered(active_send, active_reply, job_end) and ordered(active_send, active_handled, job_end)),
            "A_received_active_reply": any(m["key"] == "during-work-ack" and m["handled_at"] for m in messages),
            "B_confirmed_idle_before_second_message": bool(ordered(idle, idle_send)),
            "idle_message_triggered_wake_and_handling": bool(ordered(idle_send, wake, idle_handled)),
            "B_replied_after_wake": bool(ordered(wake, idle_reply)),
            "A_received_idle_reply": any(m["key"] == "idle-wake-ack" and m["handled_at"] for m in messages),
        }
        report = {"verified_at": stamp(), "passed": all(checks.values()), "checks": checks,
                  "events": len(events), "messages": len(messages),
                  "driver": self.state["driver"], "simulation": self.state["simulation"],
                  "agents": self.state["agents"]}
        self.state["verification"] = report
        write_json(self.directory / "verification.json", report)
        self._event("VERIFICATION", "driver", {"passed": report["passed"], "checks": checks})
        rows = ["# Codex 双会话通信测试", "", f"全部通过：{report['passed']}", "",
                "本轮未连接机械臂；后台工作为真实计时的独立模拟任务。",
                "会话创建与空闲唤醒由当前研发会话的 Codex 应用工具调度，不是无人值守后台守护程序。", "",
                "## 检查", ""]
        rows += [f"- {'PASS' if ok else 'FAIL'} — {name}" for name, ok in checks.items()]
        rows += ["", "## 会话", ""]
        rows += [f"- {a}：{v['thread_id']}；实际模型 {v['verified_model']}；实际推理档 {v['verified_effort']}" for a, v in self.state["agents"].items()]
        rows += ["", "## 完整时间线", "", "| 序号 | UTC 时间 | 来源 | 事件 | 详情 |", "|---|---|---|---|---|"]
        rows += [f"| {e['seq']} | {e['timestamp']} | {e['actor']} | {e['kind']} | {json.dumps(e['details'], ensure_ascii=False).replace('|', '/')} |" for e in events]
        (self.directory / "report.md").write_text("\n".join(rows) + "\n")
        export_dashboard(self.directory, self.snapshot())
        return report


def make_handler(pilot):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def respond(self, code, body, kind="application/json"):
            raw = body.encode() if isinstance(body, str) else json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", kind + "; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            if self.path == "/":
                return self.respond(200, (ROOT / "assets/session_mailbox_dashboard.html").read_text(), "text/html")
            if self.path == "/api/state":
                return self.respond(200, pilot.snapshot())
            self.respond(404, {"error": "not found"})

        def do_POST(self):
            if self.path != "/api/op":
                return self.respond(404, {"error": "not found"})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length < 65536:
                    raise ValueError("Invalid body size")
                body = json.loads(self.rfile.read(length))
                answer = pilot.dispatch(body["op"], body.get("agent", "driver"), body.get("data", {}))
                self.respond(200, answer)
            except (ValueError, KeyError, TypeError) as error:
                self.respond(400, {"ok": False, "error": str(error)})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["serve", "call", "export"])
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--op", default="state")
    parser.add_argument("--agent", default="driver")
    parser.add_argument("--data", default="{}")
    args = parser.parse_args()
    if args.mode == "export":
        path = export_dashboard(args.run_dir, json.loads((args.run_dir / "state.json").read_text()))
        print(str(path.resolve()))
    elif args.mode == "serve":
        pilot = Pilot(args.run_dir)
        server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(pilot))
        connection = {"url": f"http://127.0.0.1:{server.server_address[1]}", "run_id": pilot.state["run_id"]}
        write_json(pilot.directory / "connection.json", connection)
        print(json.dumps(connection), flush=True)

        def ticker():
            last_tick = -1
            while True:
                time.sleep(0.25)
                with pilot.condition:
                    pilot._finish_due()
                    job = pilot.state["job"]
                    if job and job["status"] == "RUNNING":
                        tick = int((time.time() - job["starts_epoch"]) // 10)
                        if tick != last_tick:
                            last_tick = tick
                            pilot._event("JOB_PROGRESS", "B", {"elapsed_s": round(time.time() - job["starts_epoch"], 1), "job_id": job["id"]})
        threading.Thread(target=ticker, daemon=True).start()
        server.serve_forever()
    else:
        connection = json.loads((args.run_dir / "connection.json").read_text())
        request = Request(connection["url"] + "/api/op", method="POST",
            headers={"Content-Type": "application/json"},
            data=json.dumps({"op": args.op, "agent": args.agent, "data": json.loads(args.data)}).encode())
        try:
            with urlopen(request, timeout=30) as response:
                print(response.read().decode())
        except HTTPError as error:
            print(error.read().decode())
            raise SystemExit(1)


if __name__ == "__main__":
    main()

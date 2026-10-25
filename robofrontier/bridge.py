"""Loopback command transport. Physics and images are produced on the GUI thread."""
from collections import OrderedDict
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import queue
import secrets
import threading
import time

from .scene import ROOT


@dataclass
class Request:
    payload: dict
    done: threading.Event = field(default_factory=threading.Event)
    result: dict | None = None
    deadline: float = field(default_factory=lambda: time.monotonic() + 12)


class Bridge:
    def __init__(self):
        self.queue = queue.Queue(maxsize=8)
        self.token = secrets.token_urlsafe(24)
        self.cache = OrderedDict()
        self.lock = threading.Lock()
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_POST(self):
                if self.path != "/action" or self.headers.get("X-Robot-Token") != bridge.token:
                    self.send_error(403)
                    return
                try:
                    size = int(self.headers.get("Content-Length", "0"))
                    if not 0 < size <= 8192:
                        raise ValueError("Invalid request size")
                    payload = json.loads(self.rfile.read(size))
                    request_id = payload.get("request_id")
                    if not isinstance(request_id, str) or not request_id:
                        raise ValueError("request_id is required")
                    with bridge.lock:
                        job = bridge.cache.get(request_id)
                        if job and job.payload != payload:
                            raise ValueError("request_id was reused with different parameters")
                        if job is None:
                            job = Request(payload)
                            bridge.queue.put_nowait(job)
                            bridge.cache[request_id] = job
                            while len(bridge.cache) > 256:
                                bridge.cache.popitem(last=False)
                    if not job.done.wait(15):
                        answer = {"error": "Request timed out. Reuse request_id to check; do not resend as a new action."}
                    else:
                        answer = job.result
                    content = json.dumps(answer).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(content)))
                    self.end_headers()
                    self.wfile.write(content)
                except (ValueError, TypeError, queue.Full) as error:
                    self.send_error(400, str(error))
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.path = ROOT / ".robot-bridge.json"
        fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w") as output:
            json.dump({"url": f"http://127.0.0.1:{self.server.server_port}/action", "token": self.token, "pid": os.getpid()}, output)

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.path.unlink(missing_ok=True)

"""Episode traces and bounded, asynchronous MP4 encoding."""
from datetime import datetime
import json
from pathlib import Path
import queue
import shutil
import subprocess
import threading
from zoneinfo import ZoneInfo

import numpy as np

from .scene import ROOT


def probe_video(path):
    """Verify headers and decode one frame; not a certification of every frame."""
    path=Path(path)
    if not path.is_file():return {'readable':False,'reason':'Video file missing'}
    ffprobe,ffmpeg=shutil.which('ffprobe'),shutil.which('ffmpeg')
    if not ffprobe or not ffmpeg:return {'readable':None,'reason':'Video verification tools unavailable'}
    try:
        result=subprocess.run([ffprobe,'-v','error','-select_streams','v:0','-show_entries',
            'stream=width,height:format=duration','-of','json',str(path)],capture_output=True,timeout=10)
        if result.returncode:return {'readable':False,'reason':result.stderr.decode(errors='replace')[:400]}
        info=json.loads(result.stdout);stream=info.get('streams',[{}])[0]
        width,height=stream.get('width',0),stream.get('height',0)
        if not width or not height:return {'readable':False,'reason':'No video stream'}
        decoded=subprocess.run([ffmpeg,'-v','error','-i',str(path),'-frames:v','1',
            '-f','rawvideo','-pix_fmt','rgb24','pipe:1'],capture_output=True,timeout=10)
        if decoded.returncode or len(decoded.stdout)!=width*height*3:
            return {'readable':False,'reason':'First frame could not be decoded'}
        duration=info.get('format',{}).get('duration')
        return {'readable':True,'duration_s':float(duration) if duration else None,'width':width,'height':height}
    except (OSError,ValueError,IndexError,subprocess.TimeoutExpired) as error:
        return {'readable':None,'reason':str(error)[:400]}


class Episode:
    def __init__(self, simulation):
        stamp = datetime.now(ZoneInfo("Asia/Hong_Kong")).strftime("%Y%m%d-%H%M%S-%f")
        self.path = ROOT / "runs" / stamp
        self.path.mkdir(parents=True)
        self.trace = (self.path / "trace.jsonl").open("w")
        (self.path / "metadata.json").write_text(json.dumps(simulation.metadata(), indent=2) + "\n")
        (self.path / "scene.xml").write_text(simulation.xml)
        np.savez(self.path / "initial.npz", state=simulation.initial_state,
                 q_ref=simulation.q_ref, tcp_ref=simulation.tcp_ref,
                 rotation_ref=simulation.rotation_ref, ref_velocity=simulation.ref_velocity,
                 gripper_ref=simulation.gripper_ref)

    def event(self, kind, **values):
        self.trace.write(json.dumps({"event": kind, **values}, ensure_ascii=False) + "\n")
        self.trace.flush()

    def sample(self, simulation, wall_time, action):
        self.event("sample", wall_time_s=wall_time, action=action,
                   observation=simulation.observe(oracle=True),
                   integration_state=simulation.integration_state().tolist(),
                   q_ref=simulation.q_ref.tolist())

    def close(self):
        self.trace.close()


class Video:
    def __init__(self, path, width, height, fps):
        executable = shutil.which("ffmpeg")
        if not executable:
            raise RuntimeError("FFmpeg not installed; install it to use F9 recording")
        self.path = Path(path)
        self.stderr = self.path.with_suffix(".ffmpeg.log").open("w")
        self.process = subprocess.Popen([executable, "-hide_banner", "-loglevel", "error", "-y",
            "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}", "-r", str(fps),
            "-i", "-", "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "21",
            "-tune", "zerolatency", "-g", str(max(1,round(fps))), "-sc_threshold", "0",
            "-pix_fmt", "yuv420p", "-movflags", "+frag_keyframe+empty_moov+default_base_moof",
            "-frag_duration", "1000000", "-flush_packets", "1", str(self.path)],
            stdin=subprocess.PIPE, stderr=self.stderr)
        self.queue = queue.Queue(maxsize=12)
        self.error = None
        self.dropped = 0
        self.frames = 0
        self.closed_result = None
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def _write(self):
        try:
            while True:
                data = self.queue.get()
                if data is None:
                    break
                self.process.stdin.write(data)
                self.frames += 1
        except (OSError, ValueError) as error:
            self.error = str(error)
        finally:
            try:self.process.stdin.close()
            except (OSError,ValueError) as error:self.error=self.error or str(error)

    def frame(self, pixels, block=False):
        if self.error or self.closed_result is not None:
            return False
        try:
            if block:self.queue.put(pixels.tobytes(),timeout=10)
            else:self.queue.put_nowait(pixels.tobytes())
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def close(self):
        if self.closed_result is not None:return self.closed_result
        if self.thread.is_alive():
            try:
                self.queue.put(None, timeout=3)
            except queue.Full:
                self.process.kill()
                self.error = "Encoder stalled"
        self.thread.join(timeout=5)
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
            self.error = "Encoder did not finish in time"
        if self.process.returncode:
            self.error = self.error or f"FFmpeg exited {self.process.returncode}"
        self.stderr.close()
        verification=probe_video(self.path)
        if verification['readable'] is not True:self.error=self.error or 'Video readability not verified: '+verification.get('reason','unknown')
        self.closed_result={"path": str(self.path), "frames": self.frames, "dropped_frames": self.dropped,
            "error": self.error,"encoding_complete":self.process.returncode==0 and self.error is None,
            **verification,'verification_scope':'Headers and first decoded frame; abrupt loss may discard the unfinished tail.'}
        return self.closed_result

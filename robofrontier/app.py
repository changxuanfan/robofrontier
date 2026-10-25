"""Small native MuJoCo window; all physics and OpenGL live on this thread."""
import argparse
import json
from pathlib import Path
import queue
import time

import glfw
import mujoco
import numpy as np
from PIL import Image

from .recording import Episode, Video
from .bridge import Bridge
from .sim import Simulation
from .scene import ROOT


class App:
    def __init__(self, args):
        self.args = args
        self.sim = Simulation(args.config, blind=args.blind)
        self.bridge = None
        self.remote = None
        self.action_count = 0
        self.observation_count = 0
        self.keys = set()
        self.paused = False
        self.video = None
        self.video_number = 0
        self.episode = Episode(self.sim)
        self.started = time.monotonic()
        self.record_next = self.started
        self.message = "Pick a cube, then lift it. TCP = yellow dot."
        self.show_help = True
        self.screenshot_pending = False
        self.has_focus = True
        self.run()

    def key(self, window, key, scancode, action, modifiers):
        if action == glfw.RELEASE:
            self.keys.discard(key)
            return
        if action != glfw.PRESS:
            return
        if self.remote:
            if key in (glfw.KEY_ESCAPE, glfw.KEY_SPACE, glfw.KEY_BACKSPACE):
                self.finish_remote("interrupted_by_human")
            else:
                self.message = "Agent action running; SPACE interrupts."
                return
        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(window, True)
        elif key == glfw.KEY_SPACE:
            self.paused = not self.paused
            self.keys.clear()
            self.sim.stop()
            self.message = "PHYSICS PAUSED" if self.paused else "Physics running — holding current pose"
            self.episode.event("pause", paused=self.paused, sim_time_s=self.sim.data.time)
        elif key == glfw.KEY_TAB:
            self.keys.clear()
            self.sim.set_mode("joint" if self.sim.mode == "tcp" else "tcp")
            self.episode.event("mode", mode=self.sim.mode, sim_time_s=self.sim.data.time)
        elif key in (glfw.KEY_LEFT_BRACKET, glfw.KEY_RIGHT_BRACKET):
            self.sim.speed = int(np.clip(self.sim.speed + (1 if key == glfw.KEY_RIGHT_BRACKET else -1), 0, 2))
        elif key == glfw.KEY_O:
            self.sim.set_gripper(True)
        elif key == glfw.KEY_P:
            self.sim.set_gripper(False)
        elif key == glfw.KEY_BACKSPACE:
            self.stop_video()
            self.episode.close()
            self.sim.reset()
            self.keys.clear()
            self.episode = Episode(self.sim)
            self.started = time.monotonic()
            self.message = "Initial scene restored. New episode."
        elif key == glfw.KEY_F9:
            if self.video:
                self.stop_video()
            else:
                try:
                    self.video_number += 1
                    self.video = Video(self.episode.path / f"recording-{self.video_number:02d}.mp4", 1280, 800, self.sim.config["video_fps"])
                    self.record_next = time.monotonic()
                    self.message = "Recording to " + self.video.path.name
                    self.episode.event("record_start", sim_time_s=self.sim.data.time, path=str(self.video.path))
                except RuntimeError as error:
                    self.message = str(error)
        elif key == glfw.KEY_F8:
            self.screenshot_pending = True
        elif key == glfw.KEY_F1:
            self.show_help = not self.show_help
        elif key == glfw.KEY_C:
            self.camera.fixedcamid = 1 - self.camera.fixedcamid
            self.message = "Camera changed. Arrows still follow WORLD axes."
        else:
            self.keys.add(key)

    def focus(self, window, focused):
        self.has_focus = bool(focused)
        if not focused:
            self.keys.clear()
            if not self.remote:
                self.sim.stop()
            # Keep gravity/contacts running; only cancel robot motion requests.
            self.message = "Window unfocused — robot holding; physics still running"
            self.episode.event("focus_lost", sim_time_s=self.sim.data.time)

    def input(self):
        if self.remote:
            command = self.remote["command"]
            if self.sim.data.time < self.remote["motion_end"]:
                return command["direction"], command["yaw"], [0.0] * 7
            return [0.0] * 3, 0.0, [0.0] * 7
        def axis(positive, negative):
            return float(positive in self.keys) - float(negative in self.keys)
        if self.sim.mode == "tcp":
            direction = [axis(glfw.KEY_DOWN, glfw.KEY_UP), axis(glfw.KEY_RIGHT, glfw.KEY_LEFT), axis(glfw.KEY_W, glfw.KEY_S)]
            yaw = axis(glfw.KEY_D, glfw.KEY_A)
            return direction, yaw, [0.0] * 7
        pairs = [(glfw.KEY_Q, glfw.KEY_A), (glfw.KEY_W, glfw.KEY_S), (glfw.KEY_E, glfw.KEY_D),
                 (glfw.KEY_R, glfw.KEY_F), (glfw.KEY_T, glfw.KEY_G), (glfw.KEY_Y, glfw.KEY_H), (glfw.KEY_U, glfw.KEY_J)]
        return [0.0] * 3, 0.0, [axis(*pair) for pair in pairs]

    def capture_observation(self, cameras="both"):
        """Capture RGB plus robot-only state on the simulation/render thread."""
        self.observation_count += 1
        paths = {}
        old_camera = self.camera.fixedcamid
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.context)
        viewport = mujoco.MjrRect(0, 0, 1280, 800)
        for name in (["front", "side"] if cameras == "both" else [cameras]):
            self.camera.fixedcamid = self.sim.model.camera(name).id
            self.draw(viewport)
            path = self.episode.path / f"rgb-{self.observation_count:04d}-{name}.png"
            Image.fromarray(self.read_pixels(viewport)).save(path)
            paths[name] = str(path)
        self.camera.fixedcamid = old_camera
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, self.context)
        return {"observation_id": self.observation_count, "robot": self.sim.observe(False),
                "rgb": paths, "observation_mode": "RGB + robot proprioception; no object truth or depth"}

    def reply(self, job, result):
        result = {"request_id": job.payload["request_id"], **result}
        # This audit contains exactly the policy-visible request and response.
        # Internal physics traces are separate and never returned by the bridge.
        with (self.episode.path / "policy.jsonl").open("a") as audit:
            audit.write(json.dumps({"request": job.payload, "response": result}) + "\n")
        job.result = result
        job.done.set()
        self.episode.event("tool_result", **result)

    def finish_remote(self, status="completed"):
        remote = self.remote
        self.remote = None
        self.keys.clear()
        self.message = f"Agent action {self.action_count}: {status}. Waiting for next RGB decision."
        self.reply(remote["job"], {"execution_status": status, "sim_start_s": remote["start"],
                                  **self.capture_observation()})

    def poll_bridge(self):
        if not self.bridge:
            return
        if self.remote:
            if self.sim.data.time >= self.remote["finish"]:
                self.finish_remote()
            else:
                return
        try:
            job = self.bridge.queue.get_nowait()
        except queue.Empty:
            return
        payload = job.payload
        self.episode.event("tool_request", sim_time_s=self.sim.data.time, **payload)
        try:
            if time.monotonic() > job.deadline:
                raise ValueError("Request expired before execution")
            kind = payload["type"]
            if kind == "observe":
                camera = payload.get("camera", "both")
                if camera not in ("front", "side", "both"):
                    raise ValueError("Unknown camera")
                self.reply(job, self.capture_observation(camera))
            elif kind == "record":
                if payload.get("state") not in ("start", "stop"):
                    raise ValueError("Recording state must be start or stop")
                start = payload.get("state") == "start"
                if start != bool(self.video):
                    self.key(self.window, glfw.KEY_F9, 0, glfw.PRESS, 0)
                self.reply(job, {"recording": bool(self.video), "video": str(self.video.path) if self.video else None})
            elif kind == "stop":
                self.sim.stop()
                self.reply(job, self.capture_observation())
            elif kind in ("move", "gripper", "wait"):
                if self.paused:
                    raise ValueError("Physics is paused; resume before sending an action")
                duration = float(payload.get("duration", 1.5 if kind == "gripper" else 1.0))
                max_duration = 4.0 if kind == "move" else 2.0
                if not np.isfinite(duration) or not 0.1 <= duration <= max_duration:
                    raise ValueError(f"Duration must be 0.1..{max_duration:g} s for {kind}")
                delta = np.array([float(payload.get(f"d{x}", 0)) for x in "xyz"])
                yaw = float(payload.get("yaw_deg", 0))
                if not np.isfinite(delta).all() or not np.isfinite(yaw):
                    raise ValueError("Finite numbers required")
                speed_names = ("slow", "normal", "fast")
                speed_name = payload.get("speed", "normal")
                if speed_name not in speed_names:
                    raise ValueError("Speed must be slow, normal, or fast")
                speed_index = speed_names.index(speed_name)
                speed = self.sim.config["linear_speed_m_s"][speed_index]
                max_move = speed * duration
                if np.linalg.norm(delta) > max_move + 1e-6 or abs(yaw) > 10:
                    raise ValueError(f"Move exceeds {speed_name} limit ({100 * max_move:.1f} cm and 10 deg)")
                if kind != "move" and (np.any(delta) or yaw):
                    raise ValueError("Only move actions accept motion parameters")
                direction = delta / (duration * speed)
                yaw_speed = np.deg2rad(self.sim.config["joint_speed_deg_s"][speed_index])
                yaw_input = np.deg2rad(yaw) / (duration * yaw_speed)
                if np.linalg.norm(direction) > 1.0001 or abs(yaw_input) > 1:
                    raise ValueError(f"Requested move exceeds the {speed_name} speed")
                self.keys.clear()
                if self.sim.mode != "tcp":
                    self.sim.set_mode("tcp")
                self.sim.speed = speed_index
                if kind == "gripper":
                    if payload.get("state") not in ("open", "close"):
                        raise ValueError("Gripper state must be open or close")
                    self.sim.set_gripper(payload["state"] == "open")
                self.action_count += 1
                self.remote = {"job": job, "start": self.sim.data.time,
                               "motion_end": self.sim.data.time + duration,
                               "finish": self.sim.data.time + duration + 0.35,
                               "command": {"direction": direction.tolist(), "yaw": yaw_input}}
                self.message = f"RGB agent action {self.action_count}: {kind} {delta.round(4).tolist()} m"
            else:
                raise ValueError("Unknown action type; no object-state endpoint exists")
        except (ValueError, TypeError, KeyError) as error:
            self.reply(job, {"error": str(error)})

    def stop_video(self):
        if self.video:
            result = self.video.close()
            self.episode.event("record_stop", sim_time_s=self.sim.data.time, **result)
            self.message = "Video saved: " + self.video.path.name if not result["error"] else "Video error: " + result["error"]
            print(json.dumps(result), flush=True)
            self.video = None

    def overlay(self, viewport):
        observation = self.sim.observe()
        names = ["SLOW", "NORMAL", "FAST"]
        title = f"ROBOFRONTIER   |   {self.sim.mode.upper()} MODE   |   {names[self.sim.speed]}   |   "
        title += "PAUSED" if self.paused else "LIVE PHYSICS"
        if self.video:
            title += "   |   REC"
        top = [title, f"Sim {self.sim.data.time:7.2f}s   TCP (m): " + "  ".join(f"{x:+.3f}" for x in observation["tcp_m"]),
               f"Grip {1000 * observation['gripper_width_m']:.1f} mm   TCP speed {100 * self.sim.config['linear_speed_m_s'][self.sim.speed]:g} cm/s   Joint speed {self.sim.config['joint_speed_deg_s'][self.sim.speed]:g} deg/s",
               f"Contact solver: NoSlip {self.sim.model.opt.noslip_iterations}   |   Gravity ON",
               self.sim.status, self.message]
        mujoco.mjr_overlay(mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                           viewport, "\n".join(top), "", self.context)
        if self.show_help:
            if self.sim.mode == "tcp":
                help_text = "Arrows: tabletop (UP=-X, RIGHT=+Y)\nW / S: up / down    A / D: yaw\n"
            else:
                help_text = "Joint + / - : J1 Q/A  J2 W/S  J3 E/D  J4 R/F\nJ5 T/G  J6 Y/H  J7 U/J   (multiple keys allowed)\n"
            help_text += "O / P: open / close    [ / ]: speed\nTAB: TCP / joints    SPACE: pause physics\nBACKSPACE: reset    F9: record    F8: screenshot\nC: front / side view    F1: help    ESC: exit"
            mujoco.mjr_overlay(mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                               viewport, help_text, "", self.context)
        stats = []
        for i, q in enumerate(np.rad2deg(observation["joint_rad"])):
            velocity = np.rad2deg(observation["joint_velocity_rad_s"][i])
            stats.append(f"J{i+1} {q:+7.1f} deg  {velocity:+6.1f} deg/s")
        mujoco.mjr_overlay(mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                           viewport, "Joint feedback\n" + "\n".join(stats), "", self.context)

    def draw(self, viewport):
        mujoco.mjv_updateScene(self.sim.model, self.sim.data, self.options, None, self.camera,
                              mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        # Render visual meshes, not their overlapping collision meshes.
        mujoco.mjr_render(viewport, self.scene, self.context)
        self.overlay(viewport)

    def read_pixels(self, viewport):
        pixels = np.empty((viewport.height, viewport.width, 3), dtype=np.uint8)
        mujoco.mjr_readPixels(pixels, None, viewport, self.context)
        return np.ascontiguousarray(pixels[::-1])

    def run(self):
        if not glfw.init():
            raise RuntimeError("Could not initialize GLFW. Launch in the local desktop session.")
        glfw.window_hint(glfw.VISIBLE, not self.args.hidden)
        width, height = self.sim.config["window_size"]
        window = glfw.create_window(width, height, "RoboFrontier — Panda Keyboard Demo", None, None)
        if not window:
            glfw.terminate()
            raise RuntimeError("Could not create OpenGL window")
        glfw.make_context_current(window)
        self.window = window
        glfw.swap_interval(1)
        glfw.set_key_callback(window, self.key)
        glfw.set_window_focus_callback(window, self.focus)
        self.scene = mujoco.MjvScene(self.sim.model, maxgeom=2000)
        self.context = mujoco.MjrContext(self.sim.model, mujoco.mjtFontScale.mjFONTSCALE_150)
        self.options = mujoco.MjvOption()
        self.options.geomgroup[3] = 0
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.camera.fixedcamid = self.sim.model.camera(self.args.camera).id
        self.context.offWidth = 1280
        self.context.offHeight = 800
        if self.args.bridge:
            self.bridge = Bridge()
        self.started = time.monotonic()
        if self.args.record:
            self.key(window, glfw.KEY_F9, 0, glfw.PRESS, 0)
        accumulator = 0.0
        previous = time.monotonic()
        next_sample = 0.0
        print(f"RoboFrontier ready. Episode: {self.episode.path}", flush=True)
        try:
            while not glfw.window_should_close(window):
                now = time.monotonic()
                if self.args.seconds and now - self.started >= self.args.seconds:
                    break
                elapsed = now - previous
                previous = now
                glfw.poll_events()
                self.poll_bridge()
                if not self.paused:
                    accumulator += min(elapsed, 0.10)
                    while accumulator >= self.sim.config["control_dt_s"]:
                        direction, yaw, joints = self.input()
                        self.sim.control(direction, yaw, joints)
                        accumulator -= self.sim.config["control_dt_s"]
                else:
                    accumulator = 0
                wall = max(0.0, now - self.started)
                if now >= next_sample:
                    direction, yaw, joints = self.input()
                    self.episode.sample(self.sim, wall, {"direction": direction, "yaw": yaw, "joints": joints,
                                                        "gripper_goal": self.sim.gripper_goal, "paused": self.paused})
                    next_sample = now + 0.10
                width, height = glfw.get_framebuffer_size(window)
                if width and height:
                    viewport = mujoco.MjrRect(0, 0, width, height)
                    self.draw(viewport)
                    if self.screenshot_pending or (self.args.screenshot and wall > 0.5):
                        path = Path(self.args.screenshot) if self.args.screenshot else self.episode.path / f"screenshot-{self.sim.steps:06d}.png"
                        path.parent.mkdir(parents=True, exist_ok=True)
                        Image.fromarray(self.read_pixels(viewport)).save(path)
                        self.screenshot_pending = False
                        self.args.screenshot = None
                        self.message = "Screenshot saved: " + path.name
                    if self.video and now >= self.record_next:
                        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.context)
                        video_viewport = mujoco.MjrRect(0, 0, 1280, 800)
                        self.draw(video_viewport)
                        accepted = self.video.frame(self.read_pixels(video_viewport))
                        self.episode.event("video_frame", wall_time_s=wall, sim_time_s=self.sim.data.time,
                                           accepted=accepted, frame=self.video.frames + self.video.queue.qsize())
                        self.record_next += 1 / self.sim.config["video_fps"]
                        # Resample on wall time; duplicate frames for late ticks instead of changing physics dt.
                        while self.record_next <= now:
                            self.video.frame(self.read_pixels(video_viewport))
                            self.record_next += 1 / self.sim.config["video_fps"]
                        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, self.context)
                    glfw.swap_buffers(window)
                time.sleep(0.001)
        finally:
            if self.remote:
                self.finish_remote("window_closed")
            if self.bridge:
                self.bridge.close()
            self.stop_video()
            self.episode.event("exit", sim_time_s=self.sim.data.time, wall_time_s=time.monotonic() - self.started)
            self.episode.close()
            self.context.free()
            glfw.destroy_window(window)
            glfw.terminate()


def main():
    parser = argparse.ArgumentParser(description="MuJoCo Panda keyboard demo")
    parser.add_argument("--config", default=str(ROOT / "configs/noslip.json"),
                        help="Default: validated NoSlip profile. Use config.json for the original baseline.")
    parser.add_argument("--hidden", action="store_true", help="Hidden window for rendering smoke checks")
    parser.add_argument("--seconds", type=float, default=0, help="Automatically exit after this many wall seconds")
    parser.add_argument("--screenshot", help="Save one screenshot to this path")
    parser.add_argument("--record", action="store_true", help="Start recording immediately")
    parser.add_argument("--bridge", action="store_true", help="Enable loopback RGB + proprioception actions")
    parser.add_argument("--blind", action="store_true", help="Randomize objects without exposing their coordinates to the policy")
    parser.add_argument("--camera", choices=["front", "side"], default="front")
    App(parser.parse_args())


if __name__ == "__main__":
    main()

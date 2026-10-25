"""Real encoder regression: a complete prefix survives an encoder crash."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest

import numpy as np
from robofrontier.recording import Video


@unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'FFmpeg tools required')
class RecordingTests(unittest.TestCase):
    def feed(self,video,count):
        frame=np.zeros((64,64,3),dtype=np.uint8);deadline=time.monotonic()+5
        for i in range(count):
            frame[:]=[i*7%256,i*3%256,120]
            while not video.frame(frame):
                self.assertIsNone(video.error)
                self.assertLess(time.monotonic(),deadline);time.sleep(.005)
        while video.frames<count:
            self.assertIsNone(video.error)
            self.assertLess(time.monotonic(),deadline);time.sleep(.005)

    def assert_decodable_prefix(self,path):
        p=subprocess.run([shutil.which('ffmpeg'),'-v','error','-i',str(path),'-frames:v','1',
            '-f','rawvideo','-pix_fmt','rgb24','pipe:1'],capture_output=True,timeout=10)
        self.assertEqual(p.returncode,0,p.stderr.decode()[:300]);self.assertEqual(len(p.stdout),64*64*3)

    def test_clean_close_verifies_video_and_is_idempotent(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'normal.mp4';v=Video(path,64,64,10)
            try:
                self.feed(v,30);result=v.close()
                self.assertTrue(result['readable']);self.assertTrue(result['encoding_complete'])
                self.assertEqual(v.close(),result)
                self.assert_decodable_prefix(path)
            finally:
                if v.process.poll() is None:v.process.kill();v.process.wait()

    def test_encoder_kill_keeps_complete_fragments_readable_before_close(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'crash.mp4';v=Video(path,64,64,10)
            try:
                self.feed(v,60);deadline=time.monotonic()+1
                while time.monotonic()<deadline:
                    if path.exists() and path.read_bytes().count(b'moof')>=2:break
                    time.sleep(.01)
                v.process.kill();v.process.wait()
                # No graceful close is allowed to repair the file before this.
                self.assert_decodable_prefix(path)
                result=v.close();self.assertTrue(result['readable'])
                self.assertFalse(result['encoding_complete'])
            finally:
                if v.process.poll() is None:v.process.kill();v.process.wait()
                v.close()


if __name__=='__main__':unittest.main()

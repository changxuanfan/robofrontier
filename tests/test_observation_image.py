import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

import mujoco
from PIL import Image

from robofrontier.observation_image import DEFAULT_PROFILE, LEGACY_PROFILE, observation_profile
from robofrontier.pipeline import Service
from robofrontier.run_setup import prepare_run, verify_inputs, write_json
from robofrontier.task_config import load_harness


class ObservationImageTests(unittest.TestCase):
    def test_frozen_default_legacy_fallback_and_invalid_profile(self):
        self.assertEqual((observation_profile({})['format'], observation_profile({})['width']), ('PNG', 1280))
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            new = prepare_run(root/'new')
            cfg = json.loads((new/'private/inputs/harness.json').read_text())
            public = json.loads((new/'public/inputs/task.json').read_text())
            self.assertEqual(cfg['observation_image_profile'], DEFAULT_PROFILE)
            self.assertEqual(public['observation_image'], {'width':896, 'height':560, 'format':'JPEG', 'quality':85})
            self.assertEqual(verify_inputs(new), [])
            cfg = load_harness(); cfg['observation_image_profile'] = LEGACY_PROFILE
            write_json(root/'old-config.json', cfg)
            legacy = prepare_run(root/'legacy-profile', harness_path=root/'old-config.json')
            public = json.loads((legacy/'public/inputs/task.json').read_text())
            self.assertEqual(public['observation_image'], {'width':1280, 'height':800, 'format':'PNG'})
            self.assertEqual(verify_inputs(legacy), [])
            cfg['observation_image_profile']='not-a-profile';write_json(root/'invalid.json', cfg)
            with self.assertRaises(ValueError):prepare_run(root/'invalid', harness_path=root/'invalid.json')
            self.assertFalse((root/'invalid').exists())

    def test_capture_saves_valid_images_for_both_frozen_profiles(self):
        model=mujoco.MjModel.from_xml_string("<mujoco><visual><global offwidth='1280' offheight='800'/></visual><worldbody><camera name='top' pos='0 0 2'/><geom type='sphere' size='.1'/></worldbody></mujoco>")
        data=mujoco.MjData(model);mujoco.mj_forward(model,data)
        with tempfile.TemporaryDirectory() as tmp, mujoco.Renderer(model,width=1280,height=800) as renderer:
            root=Path(tmp);(root/'public/observations').mkdir(parents=True)
            world=SimpleNamespace(model=model,data=data,time=0,arms={})
            service=SimpleNamespace(obs=0,directory=root,world=world,context=renderer._mjr_context,
                scene=renderer._scene,options=mujoco.MjvOption(),offscreen_context=True,
                perf={'capture_wall_s':0.,'captures':0})
            for name,size,fmt,suffix in ((DEFAULT_PROFILE,(896,560),'JPEG','.jpg'),(LEGACY_PROFILE,(1280,800),'PNG','.png')):
                service.observation_image=observation_profile({'observation_image_profile':name})
                result=Service.capture(service,['top']);path=Path(result['images']['top'])
                self.assertEqual(path.suffix,suffix)
                with Image.open(path) as image:
                    image.load();self.assertEqual(image.size,size);self.assertEqual(image.format,fmt)

    def test_dial_calibration_copy_keeps_frozen_original(self):
        from robofrontier.task_families import prepare
        with tempfile.TemporaryDirectory() as tmp:
            run=prepare('dial-best5-n04-d1',Path(tmp)/'dial')
            public=json.loads((run/'public/inputs/task.json').read_text())
            original=json.loads((run/'public/inputs/family/camera-calibration.json').read_text())
            derived=json.loads((run/public['camera_calibration']).read_text())
            self.assertEqual(original['image_size_pixel'],[1280,800])
            self.assertEqual(derived['image_size_pixel'],[896,560])
            self.assertEqual(derived['cameras'],original['cameras'])
            self.assertEqual(verify_inputs(run),[])

if __name__ == '__main__':unittest.main()

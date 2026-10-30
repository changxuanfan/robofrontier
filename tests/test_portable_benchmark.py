import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from scripts import portable_benchmark as cli


class PortableBenchmarkTests(unittest.TestCase):
    def test_platform_environment_preserves_cpu_linux_and_native_mac(self):
        mac=cli.execution_environment('Darwin',{'MUJOCO_GL':'osmesa','PYOPENGL_PLATFORM':'osmesa','PATH':'x'})
        self.assertNotIn('MUJOCO_GL',mac)
        self.assertNotIn('PYOPENGL_PLATFORM',mac)
        linux=cli.execution_environment('Linux',{})
        self.assertEqual((linux['MUJOCO_GL'],linux['LIBGL_ALWAYS_SOFTWARE']),('osmesa','true'))

    def test_single_command_prepares_home_and_submits_six_tasks(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ,{'OPENLUX_API_KEY':'fake'},clear=True):
            root=Path(tmp);output=root/'batch';home=root/'home'
            def prepare(path,names,**kwargs):
                self.assertEqual(names,['openlux-terra'])
                self.assertEqual(len(kwargs['cases']),6)
                self.assertTrue(home.is_dir())
                self.assertEqual(os.environ['CODEX_HOME'],str(home))
                path.mkdir();(path/'batch.json').write_text(json.dumps({'status':'prepared_not_started'}))
            def run(path,parallel):
                self.assertEqual(parallel,6)
                (path/'batch.json').write_text(json.dumps({'status':'closed'}))
            with patch.object(sys,'argv',['portable_benchmark','--output',str(output)]), \
                 patch.object(cli,'load_env'),patch.object(cli,'batch_codex_home',return_value=home), \
                 patch.object(cli.platform,'system',return_value='Darwin'),patch.object(cli,'sandbox_probe'), \
                 patch.object(cli,'native_render_probe',return_value='ok'), \
                 patch.object(cli,'prepare_batch',side_effect=prepare),patch.object(cli,'run_batch',side_effect=run):
                self.assertEqual(cli.main(),0)
            self.assertEqual(json.loads((output/'codex-home.json').read_text())['codex_home'],str(home))


if __name__=='__main__':unittest.main()

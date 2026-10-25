"""A late import must execute the recorded bytes, even if live source changes."""
import json
import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest
from robofrontier.suite.source_snapshot import freeze_executor,replace_option


class OracleSourceSnapshotTests(unittest.TestCase):
    def test_copied_package_and_lazy_script_helper_ignore_later_edits(self):
        with TemporaryDirectory() as directory:
            root=Path(directory)/'repo';(root/'robofrontier').mkdir(parents=True);(root/'scripts').mkdir();(root/'configs').mkdir()
            (root/'robofrontier/__init__.py').write_text('')
            (root/'robofrontier/sample.py').write_text('VALUE="recorded"\n')
            (root/'scripts/helper.py').write_text('VALUE="helper-recorded"\n')
            (root/'scripts/check.py').write_text('from robofrontier.sample import VALUE\nfrom scripts.helper import VALUE as other\nprint(VALUE+":"+other)\n')
            (root/'configs/harness.json').write_text('{"profile":"before"}')
            catalog=root/'configs/catalog.json';catalog.write_text('{"cases":[]}')
            frozen=freeze_executor(root,Path(directory)/'snapshot',root/'configs/harness.json',catalog)
            (root/'robofrontier/sample.py').write_text('VALUE="changed"\n')
            (root/'scripts/helper.py').write_text('VALUE="helper-changed"\n')
            (root/'configs/harness.json').write_text('{"profile":"after"}')
            env=os.environ.copy();env['PYTHONPATH']=str(frozen)
            output=subprocess.check_output([sys.executable,str(frozen/'scripts/check.py')],cwd=root,env=env,text=True)
            self.assertEqual(output.strip(),'recorded:helper-recorded')
            self.assertEqual(json.loads((frozen/'inputs/harness.json').read_text())['profile'],'before')
            self.assertEqual((frozen/'configs').resolve(),(root/'configs').resolve())
            self.assertIn('scripts/helper.py',json.loads((frozen/'snapshot.json').read_text())['files'])

    def test_replaces_both_cli_spellings_without_dropping_other_arguments(self):
        args=['--execute','--harness=old','--catalog','old','--case','example']
        actual=replace_option(replace_option(args,'--harness','frozen-harness'),'--catalog','frozen-catalog')
        self.assertEqual(actual,['--execute','--case','example','--harness','frozen-harness','--catalog','frozen-catalog'])


if __name__=='__main__':unittest.main()

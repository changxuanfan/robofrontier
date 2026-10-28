import hashlib
import io
import json
from pathlib import Path
import tempfile
import tarfile
import unittest

from scripts.benchmark_handoff import export_assets,restore_assets,setup_assets,digest


class HandoffAssetsTests(unittest.TestCase):
    def test_pinned_offline_setup_rejects_wrong_hash_and_restores(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'source';target=root/'target'
            self.fixture(source);scene=self.fixture(target)
            archive=root/'assets.tar.gz';export_assets(source,archive,source)
            config={'release_sha256':digest(target/'configs/task_families/current.json'),
                    'panda_source_sha256':digest(target/'vendor/panda/SOURCE.json'),
                    'archive_sha256':'0'*64, 'url':'https://example.invalid/assets'}
            (target/'configs/policy-assets.json').write_text(json.dumps(config))
            scene.unlink()
            with self.assertRaisesRegex(ValueError,'SHA-256 mismatch'):setup_assets(target,archive)
            self.assertFalse(scene.exists())
            config['archive_sha256']=digest(archive)
            (target/'configs/policy-assets.json').write_text(json.dumps(config))
            self.assertEqual(setup_assets(target,archive)['status'],'restored')
            self.assertEqual(setup_assets(target)['status'],'already_complete')

    def fixture(self,root):
        def write(name,value):
            p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(value);return p
        def checksum(p):return hashlib.sha256(p.read_bytes()).hexdigest()
        scene=write('configs/environments/test/assets/table.obj','geometry')
        write('configs/environments/test/workstation.json',json.dumps({'assets':[
            {'path':'assets/table.obj','sha256':checksum(scene)}]}))
        write('configs/task_families/current.json',json.dumps({'families':{'test':{'gpt':[
            {'directory':'configs/environments/test'}]}}}))
        robot=write('vendor/panda/robot.xml','robot')
        write('vendor/panda/SOURCE.json',json.dumps({'sha256':{'robot.xml':checksum(robot)}}))
        return scene

    def test_archive_restores_exact_assets_and_does_not_export_credentials(self):
        import tarfile
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'source';target=root/'target'
            self.fixture(source);self.fixture(target)
            (source/'.env').write_text('secret-not-for-export')
            archive=root/'assets.tar.gz';report=export_assets(source,archive,source)
            self.assertEqual(report['files'],3)
            with tarfile.open(archive) as tar:self.assertNotIn('.env',tar.getnames())
            (target/'configs/environments/test/assets/table.obj').unlink()
            self.assertEqual(restore_assets(archive,target)['restored'],1)
            self.assertEqual(restore_assets(archive,target)['reused'],3)
            scene=target/'configs/environments/test/assets/table.obj';scene.write_text('edited')
            with self.assertRaisesRegex(ValueError,'Destination changed'):restore_assets(archive,target)
            self.assertEqual(scene.read_text(),'edited')

    def test_source_hash_and_release_binding_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'source';scene=self.fixture(source)
            archive=root/'assets.tar.gz';export_assets(source,archive,source)
            release=source/'configs/task_families/current.json'
            release.write_text(release.read_text()+'\n')
            with self.assertRaisesRegex(ValueError,'different task release'):restore_assets(archive,source)
            scene.write_text('changed')
            with self.assertRaisesRegex(ValueError,'Missing/changed source'):export_assets(source,root/'bad.tar.gz',source)
            self.assertFalse((root/'bad.tar.gz').exists())

    def test_released_traversal_is_rejected_before_archive_creation(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);self.fixture(root)
            (root/'configs/environments/test/workstation.json').write_text(json.dumps({'assets':[
                {'path':'../../.env','sha256':'bad'}]}))
            with self.assertRaisesRegex(ValueError,'Unsafe released asset'):export_assets(root,root/'bad.tar.gz',root)

    def test_unexpected_or_corrupt_archive_cannot_partially_restore(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'source';target=root/'target'
            self.fixture(source);scene=self.fixture(target)
            original=root/'original.tar.gz';export_assets(source,original,source)
            scene.unlink()
            for extra in (True,False):
                bad=root/('extra.tar.gz' if extra else 'corrupt.tar.gz')
                with tarfile.open(original) as src,tarfile.open(bad,'w:gz') as dst:
                    for member in src.getmembers():
                        content=src.extractfile(member).read()
                        if not extra and member.name=='vendor/panda/robot.xml':content=b'corrupt'
                        member.size=len(content);dst.addfile(member,io.BytesIO(content))
                    if extra:
                        member=tarfile.TarInfo('../outside');member.size=1
                        dst.addfile(member,io.BytesIO(b'x'))
                with self.assertRaisesRegex(ValueError,'Unexpected archive|Corrupt archive'):
                    restore_assets(bad,target)
                self.assertFalse(scene.exists())
                self.assertFalse((root/'outside').exists())


if __name__=='__main__':unittest.main()

"""Delivery defaults, asset recovery and immutable package boundaries."""
import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robofrontier.suite.mosaic_delivery import (check_gpt,export_package,
    materialize_assets,settings,verify_protected)
from robofrontier.suite.mosaic_policy_package import checksum,write
from scripts.mosaic import main


class DeliveryTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory();self.addCleanup(self.temporary.cleanup)
        self.root=Path(self.temporary.name)
        self.source=self.root/'recorded/assets/cube.obj';self.source.parent.mkdir(parents=True)
        self.source.write_bytes(b'immutable mesh')
        self.row=dict(environment='configs/environments/mosaic/r14-case',path='assets/cube.obj',
            recorded_source='recorded',sha256=checksum(self.source))

    def test_default_command_checks_gpt_without_starting_a_run(self):
        with patch('scripts.mosaic.check_delivery',return_value={'passed':True}) as check,contextlib.redirect_stdout(io.StringIO()):
            main([])
        check.assert_called_once_with('gpt',None,False)

    def test_selection_disagreement_is_rejected(self):
        path=self.root/'configs/task_families/mosaic'
        write(path/'delivery.json',dict(default_purpose='gpt',purposes={'gpt':dict(registry='r14'),'oracle':dict(registry='r13')}))
        write(path/'selection.json',dict(future_gpt_registry='r10',oracle_and_factory_replay_registry='r13'))
        with self.assertRaisesRegex(ValueError,'purpose selection'):settings(self.root)

    def test_missing_worktree_assets_restore_from_frozen_recording_and_reuse(self):
        dest=self.root/'checkout'
        result=materialize_assets(dest,root=self.root,inventory=[self.row])
        self.assertEqual(result['copied'],1)
        target=dest/self.row['environment']/self.row['path']
        self.assertEqual(target.read_bytes(),self.source.read_bytes())
        self.assertEqual(materialize_assets(dest,root=self.root,inventory=[self.row])['reused'],1)

    def test_dry_run_creates_no_checkout(self):
        dest=self.root/'dry'
        result=materialize_assets(dest,root=self.root,dry_run=True,inventory=[self.row])
        self.assertEqual(result['copied'],1);self.assertFalse(dest.exists())

    def test_conflicting_destination_and_corrupt_source_are_preserved_and_rejected(self):
        dest=self.root/'checkout';target=dest/self.row['environment']/self.row['path']
        target.parent.mkdir(parents=True);target.write_bytes(b'user file')
        with self.assertRaisesRegex(ValueError,'Conflicting destination'):
            materialize_assets(dest,root=self.root,inventory=[self.row])
        self.assertEqual(target.read_bytes(),b'user file')
        self.source.write_bytes(b'corrupt')
        with self.assertRaisesRegex(ValueError,'Source asset differs'):
            materialize_assets(self.root/'other',root=self.root,inventory=[self.row])

    def test_same_assets_share_storage_and_cache_fallback_is_supported(self):
        second=dict(self.row,environment='configs/environments/mosaic/r13-case')
        dest=self.root/'checkout'
        result=materialize_assets(dest,root=self.root,inventory=[self.row,second])
        self.assertEqual(result['linked'],1)
        self.assertEqual((dest/self.row['environment']/self.row['path']).stat().st_ino,
                         (dest/second['environment']/second['path']).stat().st_ino)
        cache=self.root/'runs/state-assets'/self.row['sha256'];cache.parent.mkdir(parents=True)
        self.source.replace(cache)
        self.assertTrue(materialize_assets(self.root/'cache-checkout',root=self.root,inventory=[self.row])['passed'])

    def test_asset_traversal_and_symlink_escape_are_rejected(self):
        for value in ['../outside','/tmp/mesh.obj','textures/mesh.obj']:
            with self.assertRaisesRegex(ValueError,'assets path'):
                materialize_assets(self.root/'checkout',root=self.root,inventory=[dict(self.row,path=value)])
        dest=self.root/'checkout';dest.mkdir();outside=self.root/'outside';outside.mkdir()
        (dest/'configs').symlink_to(outside,target_is_directory=True)
        with self.assertRaisesRegex(ValueError,'escapes repository'):
            materialize_assets(dest,root=self.root,inventory=[self.row])

    def test_public_symlink_is_rejected_before_reading_any_evidence(self):
        package=self.root/'package';package.mkdir();(package/'public').symlink_to(self.root/'recorded',target_is_directory=True)
        with self.assertRaisesRegex(ValueError,'Public symlink'):check_gpt(self.root,package)

    def test_package_export_cannot_recurse_into_its_source(self):
        with patch('robofrontier.suite.mosaic_delivery.ROOT',self.root),patch('robofrontier.suite.mosaic_delivery.settings',return_value={'purposes':{'gpt':{'package':'frozen'}}}):
            with self.assertRaisesRegex(ValueError,'outside the frozen package'):
                export_package(self.root/'frozen/nested')

    def test_protection_detects_modified_bytes_and_inventory_tampering(self):
        path=self.root/'protected.json'
        write(path,dict(files={'recorded/assets/cube.obj':dict(sha256=checksum(self.source),bytes=self.source.stat().st_size)}))
        config=dict(protected_snapshot=dict(path='protected.json',sha256=checksum(path)))
        with patch('robofrontier.suite.mosaic_delivery.ROOT',self.root),patch('robofrontier.suite.mosaic_delivery.settings',return_value=config):
            self.assertEqual(verify_protected()['protected_files'],1)
            self.source.write_bytes(b'modified')
            with self.assertRaisesRegex(ValueError,'Protected file differs'):verify_protected()
            path.write_text('{}')
            with self.assertRaisesRegex(ValueError,'Protection inventory differs'):verify_protected()


if __name__=='__main__':unittest.main()

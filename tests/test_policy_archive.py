import base64
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from scripts.archive_policy_batch import create, verify, verify_collection, transform_session, read_object


class PolicyArchiveTest(unittest.TestCase):
    def test_change_after_initial_validation_is_rejected(self):
        import hashlib
        for legacy in (False,True):
            with self.subTest(legacy=legacy), tempfile.TemporaryDirectory() as temp:
                root=Path(temp);source=root/'source'
                (source/'batch').mkdir(parents=True);(source/'sessions').mkdir()
                session=source/'sessions/a.jsonl';session.write_bytes(b'{}\n')
                (source/'batch/batch.json').write_text('{"status":"closed","jobs":[]}')
                checksum=hashlib.sha256(session.read_bytes()).hexdigest()
                report={'schema':'robofrontier.luna-collection.v1' if legacy else 'robofrontier.policy-collection.v1',
                        'engine':'codex','complete':True,'writers_stopped':True,'job_count':1,
                        'jobs':[{'roles':[{'session':'sessions/a.jsonl','sha256':checksum}]}]}
                if not legacy:
                    report['files']={p.relative_to(source).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in source.rglob('*') if p.is_file()}
                (source/'collection.json').write_text(json.dumps(report))
                def validated(*args,**kwargs):
                    result=verify_collection(*args,**kwargs)
                    session.write_bytes(b'{"late":true}\n')
                    return result
                with patch('scripts.archive_policy_batch.verify_collection',side_effect=validated):
                    with self.assertRaises(ValueError):create(source,root/'archive')
                self.assertFalse((root/'archive').exists())

    def test_typed_claude_images_and_uri_preserve_every_byte(self):
        import hashlib
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); (root/'objects').mkdir()
            image=b'\xff\xd8\xff\x00same-image'; encoded=base64.b64encode(image).decode()
            raw=(' { "text": "中文", "content" : [ '
                 '{"type":"image","data":"'+encoded+'","mimeType":"image/jpeg"},'
                 '{"type":"image","source":{"type":"base64","media_type":"image/jpeg","data":"'+encoded+'"}},'
                 '{"type":"input_image","image_url":"data:image/jpeg;base64,'+encoded+'"}'
                 ' ], "other":{"data":"do not touch"} } \r\n').encode()
            source=root/'source';source.write_bytes(raw);objects={}
            transformed=transform_session(source,root,objects)
            data=transformed.read_bytes()
            self.assertNotIn(encoded.encode(),data)
            key=hashlib.sha256(image).hexdigest()
            self.assertEqual(read_object(root,{'objects':objects},key),image)
            self.assertEqual(len(objects),1)
            self.assertIn(b'"data":"do not touch"',data)
            # Exercise the public archive/restore path, including a CLI event file.
            source_dir=root/'collection';(source_dir/'batch/private/evidence').mkdir(parents=True)
            (source_dir/'sessions').mkdir()
            for rel in ('sessions/native.jsonl','batch/private/evidence/turn.jsonl'):
                (source_dir/rel).write_bytes(raw)
            (source_dir/'batch/batch.json').write_text('{"status":"closed","jobs":[]}')
            (source_dir/'collection.json').write_text(json.dumps({'schema':'robofrontier.luna-collection.v1','complete':True,'job_count':1,'jobs':[{'roles':[{'session':'sessions/native.jsonl','sha256':hashlib.sha256(raw).hexdigest()}]}]}))
            create(source_dir,root/'archive');verify(root/'archive',root/'restore')
            for rel in ('sessions/native.jsonl','batch/private/evidence/turn.jsonl'):
                self.assertEqual((root/'restore'/rel).read_bytes(),raw)

    def test_unknown_encoding_and_interrupted_tail_retained(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'source'
            raw=b'{"data":"YXJiaXRyYXJ5"}\n{"type":"image","data":"invalid","mimeType":"image/png"}\n{"incomplete":"data:image/png;base64,A'
            source.write_bytes(raw)
            transformed=transform_session(source,root,{})
            self.assertEqual(transformed.read_bytes(),raw)

    def test_literal_archive_markers_fall_back_without_reinterpreting_text(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'source';source.write_bytes(b'"@RF_IMAGE_'+b'a'*64+b'@"\n')
            self.assertIsNone(transform_session(source,root,{}))

    def test_roundtrip_and_corruption(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); source = root/'source'; (source/'batch').mkdir(parents=True); (source/'sessions'/'a').mkdir(parents=True)
            image = b'jpeg bytes \x00\xff'; uri = b'data:image/jpeg;base64,' + base64.b64encode(image)
            raw = b'{"image":"' + uri + b'","again":"' + uri + b'"}\n'
            session = source/'sessions/a/abc.jsonl'; session.write_bytes(raw)
            import hashlib
            report = {'schema':'robofrontier.luna-collection.v1','complete':True,'job_count':1,'jobs':[{'roles':[{'session':'sessions/a/abc.jsonl','sha256':hashlib.sha256(raw).hexdigest()}]}]}
            (source/'collection.json').write_text(json.dumps(report))
            (source/'batch/batch.json').write_text(json.dumps({'status':'closed','jobs':[{'status':'closed'}]}))
            (source/'batch/motion.npz').write_bytes(b'motion\x00')
            (source/'batch/large.bin').write_bytes(b'unchanged-state\x00'*(160000))
            quoted_marker=b'{"quoted":"@RF_IMAGE_'+b'a'*64+b'@"}\n'
            (source/'batch/quoted.jsonl').write_bytes(quoted_marker)
            archive = root/'archive'; create(source, archive)
            restored = root/'restored'; verify(archive, restored)
            self.assertEqual((restored/'sessions/a/abc.jsonl').read_bytes(), raw)
            self.assertEqual((restored/'batch/motion.npz').read_bytes(), b'motion\x00')
            self.assertEqual((restored/'batch/large.bin').read_bytes(),(source/'batch/large.bin').read_bytes())
            self.assertEqual((restored/'batch/quoted.jsonl').read_bytes(),quoted_marker)
            manifest = json.loads((archive/'manifest.json').read_text())
            key = next(iter(manifest['objects'])); path = archive/'objects'/key[:2]/key
            path.write_bytes(b'bad')
            with self.assertRaises(ValueError): verify(archive, None)
            with self.assertRaises(ValueError): verify(archive, root/'bad-restore')
            self.assertFalse((root/'bad-restore').exists())

    def test_incomplete_collection_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); source = root/'source'; source.mkdir()
            (source/'collection.json').write_text(json.dumps({'schema':'robofrontier.luna-collection.v1','complete':False}))
            with self.assertRaises(ValueError): create(source, root/'archive')
            self.assertFalse((root/'archive').exists())


if __name__ == '__main__': unittest.main()

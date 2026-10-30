import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from scripts.verify_evidence_replay import verify_evidence


class EvidenceReplayInputTests(unittest.TestCase):
    def make_collection(self, root, recordings):
        source=root/'source';(source/'batch').mkdir(parents=True)
        jobs=[]
        for i,recorded in enumerate(recordings):
            case=f'case{i}';run='batch/model/'+case
            jobs.append({'run':run,'roles':[],'case_id':case})
            if recorded:
                path=source/run/'private/outputs/motion/stream.json'
                path.parent.mkdir(parents=True);path.write_text('{}')
        (source/'batch/batch.json').write_text('{"status":"closed","jobs":[]}')
        report={'schema':'robofrontier.policy-collection.v1','engine':'codex','complete':True,
                'writers_stopped':True,'job_count':len(jobs),'jobs':jobs,
                'files':{p.relative_to(source).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in source.rglob('*') if p.is_file()}}
        (source/'collection.json').write_text(json.dumps(report))
        shutil.copytree(source,root/'restored')
        return source,root/'restored'

    def test_nonexistent_or_empty_directories_cannot_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            with self.assertRaises(ValueError):verify_evidence(root/'missing',root/'missing2',root/'output')
            (root/'empty').mkdir();(root/'empty2').mkdir()
            with self.assertRaises((ValueError,FileNotFoundError)):verify_evidence(root/'empty',root/'empty2',root/'output')
            self.assertFalse((root/'output').exists())

    def test_zero_or_partial_motion_coverage_cannot_pass(self):
        for recordings in ([False],[True,False]):
            with self.subTest(recordings=recordings), tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);source,restored=self.make_collection(root,recordings)
                with self.assertRaisesRegex(ValueError,'No Motion|coverage differs'):
                    verify_evidence(source,restored,root/'output')
                self.assertFalse((root/'output').exists())

    def test_collection_cannot_be_compared_with_itself(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source,_=self.make_collection(root,[True])
            with self.assertRaises(ValueError):verify_evidence(source,source,root/'output')


if __name__=='__main__':unittest.main()

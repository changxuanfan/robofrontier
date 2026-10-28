import json
from pathlib import Path
import tempfile
import unittest

from robofrontier.claude.bootstrap import bootstrap_prompt


class BootstrapTests(unittest.TestCase):
    def fixture(self, tmp):
        run = Path(tmp) / 'run'; public = run / 'public/inputs'
        def write(name, text):
            target = public / name; target.parent.mkdir(parents=True, exist_ok=True); target.write_text(text)
        write('task.json', json.dumps({'family_materials': ['family/guide.md'], 'initial_reference_images': ['family/image.png']}))
        write('roles.json', json.dumps({'topology': 'centralized', 'roles': {'Controller': {'kind': 'controller'}}}))
        for name in ('claude-controller.md', 'policy-rules.md', 'skill/SKILL.md', 'skill/references/tools.md',
                     'skill/references/communication.md', 'skill/references/motion-check.md', 'family/guide.md'):
            write(name, 'public material: ' + name)
        private = run / 'private/secret.txt'; private.parent.mkdir(); private.write_text('not for policy')
        return run

    def test_includes_only_allowed_materials_and_public_image_instruction(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self.fixture(tmp); prompt = bootstrap_prompt(run)
            self.assertIn('public material: family/guide.md', prompt)
            self.assertIn('public material: policy-rules.md', prompt)
            self.assertIn('public_image', prompt)
            self.assertNotIn('not for policy', prompt)

    def test_rejects_private_path_symlink_and_unqualified_topology(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = self.fixture(tmp); public = run / 'public/inputs'
            task = public / 'task.json'; task.write_text(json.dumps({'family_materials': ['../private/secret.txt']}))
            with self.assertRaises(ValueError): bootstrap_prompt(run)
            task.write_text(json.dumps({'family_materials': ['family/guide.md']}))
            (public/'family/guide.md').unlink(); (public/'family/guide.md').symlink_to(run/'private/secret.txt')
            with self.assertRaises(ValueError): bootstrap_prompt(run)
            (public/'roles.json').write_text(json.dumps({'topology': 'adaptive', 'roles': {'Controller': {'kind': 'controller'}}}))
            with self.assertRaises(ValueError): bootstrap_prompt(run)

    def test_adaptive_requires_and_includes_its_reference(self):
        with tempfile.TemporaryDirectory() as tmp:
            run=self.fixture(tmp); public=run/'public/inputs'
            (public/'roles.json').write_text(json.dumps({'topology':'adaptive','roles':{'Controller':{'kind':'controller'}}}))
            with self.assertRaisesRegex(ValueError,'adaptive.md'): bootstrap_prompt(run,topology='adaptive')
            (public/'skill/references/adaptive.md').write_text('bounded role instructions')
            prompt=bootstrap_prompt(run,topology='adaptive')
            self.assertIn('bounded role instructions',prompt)
            self.assertIn('session-management tools and filesystem access are unavailable',prompt)

    def test_actual_adaptive_materials_use_mcp_report_access(self):
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            run=self.fixture(tmp); public=run/'public/inputs'
            (public/'roles.json').write_text(json.dumps({'topology':'adaptive','roles':{'Controller':{'kind':'controller'}}}))
            for name in ('SKILL.md','references/tools.md','references/communication.md',
                         'references/motion-check.md','references/adaptive.md'):
                target=public/'skill'/name; target.parent.mkdir(parents=True,exist_ok=True)
                target.write_text((root/'.agents/skills/robot-collaboration'/name).read_text())
            (public/'policy-rules.md').write_text((root/'prompts/policy-rules.md').read_text())
            prompt=bootstrap_prompt(run,topology='adaptive',startup_protocol='direct_controller_v2')
            self.assertIn('`ready` tool is optional',prompt)
            self.assertIn('Delegation is optional',prompt)
            self.assertIn('the authorized report is returned inline as delegate_report',prompt)
            self.assertNotIn('Read full reports on demand from the referenced file',prompt)
            self.assertNotIn('collisions, drops and handover history are recorded separately',prompt)

    def test_older_frozen_draft_without_shared_contract_still_loads(self):
        with tempfile.TemporaryDirectory() as tmp:
            run=self.fixture(tmp); (run/'public/inputs/policy-rules.md').unlink()
            self.assertIn('public material: family/guide.md',bootstrap_prompt(run))


if __name__ == '__main__': unittest.main()

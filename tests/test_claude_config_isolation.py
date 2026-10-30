import unittest
from pathlib import Path
from robofrontier.claude.runner import build_env

class ClaudeConfigIsolationTests(unittest.TestCase):
    def test_parallel_jobs_use_distinct_task_configuration_directories(self):
        profile={'env_key':'ANTHROPIC_API_KEY','provider':'anthropic'}
        env={'ANTHROPIC_API_KEY':'fake'}
        configs=[Path('/tmp/batch')/str(i)/'private/claude-config' for i in range(36)]
        self.assertEqual(len({build_env(profile,env,p)['CLAUDE_CONFIG_DIR'] for p in configs}),36)

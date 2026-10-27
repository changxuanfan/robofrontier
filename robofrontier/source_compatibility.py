"""Explicit compatibility of historical Oracle checks with shared integration.

The returned hash identifies the historical source, never the current file.
Current bytes and the unaffected AST must both match the reviewed receipt.
This does not certify Relay's changed scoring or produce new physical evidence.
"""
import ast
import hashlib
import json
from pathlib import Path

RECEIPT = 'configs/task_families/source-compatibility.json'


def normalized_digest(source, rule):
    tree = ast.parse(source)
    if rule == 'exclude_relay_class':
        found = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Relay']
        if len(found) != 1:
            raise ValueError('Expected one Relay class')
        tree.body.remove(found[0])
    elif rule in ('exclude_prepared_policy_guard', 'exclude_policy_service'):
        world = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'World')
        method = next(n for n in world.body if isinstance(n, ast.FunctionDef) and n.name == 'start_trial')
        expected = ast.parse("if self.task.get('execution_readiness')=='prepared_policy_trial':\n from .task_families import validate_frozen_policy\n validate_frozen_policy(self.task,self.directory)").body[0]
        method.body = [n for n in method.body if ast.dump(n) != ast.dump(expected)]
        if rule == 'exclude_policy_service':
            # CLI hosting changes only the GUI/service wrapper. World stepping,
            # robot control, task sampling and recording remain in the digest.
            service=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Service')
            tree.body.remove(service)
    else:
        raise ValueError('Unknown compatibility rule')
    return hashlib.sha256(ast.dump(tree).encode()).hexdigest()


def historical_digest(root, relative, family):
    root = Path(root)
    data = (root/relative).read_bytes()
    actual = hashlib.sha256(data).hexdigest()
    receipt = json.loads((root/RECEIPT).read_text())
    for item in receipt['files']:
        if item['path'] != relative or family not in item['families']:
            continue
        if actual == item['historical_sha256']:
            return actual
        if actual != item['integrated_sha256'] or normalized_digest(data, item['rule']) != item['unaffected_ast_sha256']:
            raise ValueError('Unreviewed shared source change: '+relative)
        return item['historical_sha256']
    return actual

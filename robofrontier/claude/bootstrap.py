"""Construct a policy bootstrap solely from frozen, allowlisted public inputs."""
import json
from pathlib import Path


MAX_BOOTSTRAP_BYTES = 400_000


def bootstrap_prompt(run_dir, *, topology='centralized', startup_protocol='ready_barrier_v1'):
    run = Path(run_dir).resolve()
    public = (run / 'public/inputs').resolve()
    task = json.loads((public / 'task.json').read_text())
    roles = json.loads((public / 'roles.json').read_text())
    if topology not in ('centralized','adaptive') or roles.get('topology') != topology or roles.get('roles', {}).get('Controller', {}).get('kind') != 'controller':
        raise ValueError('Expected the selected Controller topology')
    # No filesystem tools are exposed to the policy, so its complete textual
    # public materials must be supplied in the prompt. Images use public_image.
    paths = ['claude-controller.md', 'task.json', 'roles.json', 'skill/SKILL.md',
             'skill/references/tools.md', 'skill/references/communication.md',
             'skill/references/motion-check.md']
    if topology == 'adaptive': paths.append('skill/references/adaptive.md')
    for name in task.get('family_materials', []):
        if not isinstance(name, str): raise ValueError('Invalid public material reference')
        if name.endswith(('.md', '.json', '.txt')): paths.append(name)
    parts = []
    for relative in dict.fromkeys(paths):
        path = (public / relative).resolve()
        if not path.is_relative_to(public) or not path.is_file():
            raise ValueError('Missing or invalid public input: ' + relative)
        parts.append('--- ' + relative + ' ---\n' + path.read_text())
    text = '\n\n'.join(parts)
    if len(text.encode()) > MAX_BOOTSTRAP_BYTES:
        raise ValueError('Public inputs exceed bootstrap size limit')
    extra = ('\n\nThe adaptive role may request bounded delegates through the robot delegate operation. '
             'Native session-management tools and filesystem access are unavailable; use robot work, '
             'history, inbox and authorized messages for status and handback evidence. To read a returned '
             'delegate report, call robot work with data {"delegate_id":"Delegate_..."}; '
             'the authorized report is returned inline as delegate_report.' if topology == 'adaptive' else '')
    if startup_protocol == 'direct_controller_v2':
        extra += ('\n\nThe experiment has STARTED and the task clock is running. Read the task and '
                  'observe the scene, then decide and act. You may call ready for robot capabilities; '
                  'continue without waiting for another START.')
        private_task = run / 'private/inputs/task.json'
        protocol = json.loads(private_task.read_text()).get('run_protocol', {}) if private_task.exists() else {}
        if protocol.get('evaluation_semantics') == 'terminal_outcome_v2':
            extra += (' Terminal goal state determines completion; collisions, drops and handover history '
                      'are recorded separately. Observe and recover after problems.')
    else:
        extra += ('\n\nBefore START, inspect the task and scene, call ready, save waiting and end '
                  'the startup turn. Await the host START before task actions.')
    return text + ('\n\nTo view a task initial_reference_images entry, robot_layout_image, '
                   'or target.reference_image (such as the Mosaic target pattern), '
                   'call robot with op public_image and data {"names": ["the exact listed path"]}. '
                   'For live observations use observe with cameras and views. You have no filesystem tools. '
                   'To get detailed static geometry, call robot_model with data {"robot":"A"} '
                   '(substitute a robot ID from the task); details are returned inline.') + extra

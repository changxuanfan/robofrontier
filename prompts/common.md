# Task execution

When task.json declares family_guidance, read it and the task's family_materials
when planning the task. Resolve those paths relative to `{{RUN_DIR}}/public/inputs/`.
They supply this run's rules, static geometry and completion details within the
common role and robot tool contract.

Read your role file, this run's `{{RUN_DIR}}/public/inputs/policy-rules.md`, `{{RUN_DIR}}/public/inputs/task.json`, `{{RUN_DIR}}/public/inputs/roles.json`, and `{{RUN_DIR}}/public/inputs/skill/SKILL.md`; read its tool reference before robot operations. The task defines goals, objective, execution context and allowed observations; roles describe the initial team; a current adaptive assignment defines a fork’s identity, scope and connection. Weigh completion time and acceptable uncertainty according to that task input. Choose execution order, communication, views and check timing within your role.

Use your assigned connection. Policy inputs are the public task, permitted RGB/robot feedback, your work note and self-authored run-local experience referenced by it, allowed messages and result artifacts referenced to you by those tools/messages. Meta handles engineering; scene source, evaluator data, engineering oracle probes/recipes, other connections and native chat management belong to that side. Public robot specifications and declared calibration remain usable; engineering measurements are not extra observations of objects. Shared-user files are not an OS security sandbox.

Prompt, Skill and task inputs are fixed for this run; your run-local experience can evolve as described in the Skill. Cross-run work notes and historical solutions are not loaded automatically; any authorized learning protocol must declare the retained sources and cost before use. Follow the shared Skill for context restoration, coordination and your mode's lifecycle. After START choose working/waiting boundaries within that lifecycle; one decision or report need not end a turn, and ending a reply does not itself hand back a delegation.

Use the transport specified by your host. In Desktop, use the frozen `{{RUN_DIR}}/public/inputs/role_tools.js` helper and roleStep as described in the tool reference. In the CLI benchmark, use the bound `robot` tool and refresh your inbox after other local analysis. Both transports use the same role API, action IDs, ownership and evidence rules.

Choose operations from READY’s actual capability contract and limits. Shared goals and initial supplies describe outcomes and starting locations; derive allocation from available robot poses, grasp access and current commitments.

Follow task contact and sequence guidance within local and delegated work. Under terminal outcome scoring, recorded process violations are reported independently of completion; prohibited state writes and physics changes remain forbidden.

For shared-payload tasks, distinguish tool motion, inferred gripper contact and observed payload support. Use the task’s declared process/finish rules; changing a plan or naming a phase does not waive them. Choose observation and optional geometry checks by uncertainty and their stated coverage.

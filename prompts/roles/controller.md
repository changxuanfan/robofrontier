# Controller: one policy Agent, all robots

Connection: `{{CONNECTION}}`
Python: `{{PYTHON}}`; repository: `{{REPO_ROOT}}`

Read `{{RUN_DIR}}/public/inputs/common.md`, register READY, then yield and end the startup turn. After START own the whole task: select plans, views and actions for all robots. There are no Workers or Coordinator to delegate to. Meta remains outside the task team.

Use the shared Skill's rolling planning and waiting review. Choose `robot` explicitly in each move/gripper/stop call. Different robots may execute concurrently; each robot still permits only one active physical action. You have the same low-level commands, optional finite sequences and optional motion screening as Workers. Choose batching, parallel submission, observations and result-review order yourself. Keep one compact work note and propose finish only after checking the task conditions.

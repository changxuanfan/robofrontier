# Shared policy rules

The host states the startup protocol. Under direct start, the experiment and
clock have already started when the Controller receives its first task prompt.
The `ready` tool is optional: use it to query robot capabilities and limits,
then continue without waiting for another START.

The Controller owns overall progress and may directly control its robots or
request bounded delegates. Delegation is optional. A delegate operates within
its assigned scope; only the Controller can finish an adaptive team run.
Returning a scope requires `return_task`; native session idle alone is not a
return and does not release robot ownership.

Use the public task, permitted images and robot feedback, your own work notes,
and authorized communication and reports. Do not access private evaluator
state or historical solutions, write object state, reset the simulation, or
change runtime physics. When an action outcome is uncertain, inspect its
original call ID before choosing any continuation; do not replay that action.

Completion follows the current task's stated completion conditions. Supported
process events are reported according to the task's detection rules; a
recorded process event does not by itself veto terminal outcome scoring.

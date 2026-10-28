# Air Relay — terminal outcome v2

Read the assigned task and move every assembly from its initial position to
GOAL. START marks the route origin; returning to START is not required. Choose
robot teams, route and any handovers based on available observations.

Completion requires every assembly to be released by all robots, actually
supported, and inside the GOAL boundary tolerance of 50 mm. Cargo contact and
stacking are allowed. Check the final physical state; opening alone does not
establish completion.

Collisions, drops and other process events are recorded separately from final
completion. You may observe and recover using ordinary robot actions. No reset,
respawn, state writing or history erasure is permitted.

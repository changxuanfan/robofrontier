# Transport task reference

Read the current public task's orders, table_geometry, mechanism, completion and
READY capabilities before planning. Coordinates are in the canonical workstation
frame. The physical revision and tabletop opening are declared in that task;
use its actual support boxes rather than the outer table bounding rectangle.

Each board needs three requested source-identity items. Source and board letters
describe inventory and orders; they do not assign exclusive robot ownership.
Match the identity and visible color/shape/slot cues. The 84mm painted symbols
are identity cues; acceptance uses the separately declared physical target rules.

For L1, release each item stably within its individual target footprint. For L2,
both blades must enter the real blind slots, and the body must be seated,
released and stable. L3 has the same sockets on a passive z/roll/pitch spring
board: use the board's current orientation when reasoning about seating. A whole
item's pose and the two blade envelopes matter; merely touching a socket is not
insertion. Success includes all required items and the declared stable duration.

Plan useful concurrent work from shared goals and current observations. Compare
direct airborne handover, stable tabletop staging and waiting by complete handling
cost, occupied arms, release clearance and risk. There are no mandatory buffers,
fixed routes or fixed robot pairs. Account for the sequence from initial grasp
through reorientation, receiving grasp, insertion, jaw opening and withdrawal.
Keep previously placed items accessible and verify affected placements after contact.

Before releasing an airborne item, establish the receiver's support using allowed
RGB and robot feedback. A commanded grasp, inferred is_grasped, measured aperture
or TCP motion alone does not prove that the item is supported. When uncertain,
choose an informative observation or a bounded verification action. After a known
failure, reason from the actual scene before regrasping or transferring. A client
timeout has unknown execution semantics: query the original action ID first.

READY defines actual bounds and control limits. Optional motion screening covers
robot geometry and its stated prediction horizon; it supplies no live object,
table-contact or carried-load truth. Use it according to its coverage. Runtime
checkpoints preserve notes, not historical physics/controller states.

Policy observations remain the public task and approved images, robot feedback,
messages and run-local experience. Choose your own allocation, grasp poses and
routes. Engineering trajectories and historical private recovery recipes are not
policy observations. Display playback speed and offline removal of waiting have
no effect on the live task's clock, stable-duration rule or efficiency accounting.

# Communication contract v0.10

Meta is the experiment manager, outside the task team. Coordinator is the task planner. A Worker controls one bound arm.

Centralized mode has one Controller for all robots and no task teammates. It uses explicit robot selection and the same physical tools; Meta remains outside the policy team. No cross-session task communication is needed in that mode.

Adaptive mode routes Controller ↔ active delegates; returned scopes do not receive new work. A runtime assignment authorizes its bounded execution without a separate acceptance round. Coordinate changes to an active scope, and use return_task to hand back results and control. The small completion notice references a saved report; the Controller chooses whether to read it.

Business messages have one format: `to` (explicit recipients), `text` (natural language), optional `reply_to` (an accessible message ID). The service adds message ID, sender, submission order and time. No request/result/cancel categories are required. Text can discuss any of these subjects.

In hierarchical mode Coordinator ↔ Worker is allowed, Worker ↔ Worker is rejected. In peer mode Workers may address each other and there is no Coordinator. Meta cannot send business messages or control a robot. Global/local camera access does not grant access to another role's private mailbox.

Coordinator relays relevant decisions and readiness so Workers can coordinate without contacting forbidden peers. Peers negotiate only with affected teammates. The message format remains the same in both modes.

When assigning work or materially changing a commitment, describe the outcome, currently executable scope and earliest remaining prerequisite. On updates, identify previous prerequisites now satisfied, citing the relevant report/observation and its time or message ID; state which restrictions remain and which are replaced. Relay enabling facts with the authorization so recipients can resolve earlier conditions. Ordinary progress messages can be brief. The Skill defines standing local authority; Coordinator grants joint hierarchical scopes, while peers agree affected commitments directly. Verified conditions within scope can enable continuation without another approval; Agents still decide and call actions.

An illustrative scope: “Complete the assigned outcome. Prepare locally now; enter resource X once you verify its specified release using the agreed evidence. Continue the authorized next stage yourself; ask again if the reservation or outcome changes.” Coordinator can grant this; peers can accept it together. If a needed fact is private to another Worker in a hierarchy, Coordinator relays it. Public observable facts can instead be checked by the authorized executor. Choose the actual scopes, evidence and restrictions from the scene.

Reserve the actual conflicting resource/time and name its release milestone. Do not impose a whole-batch barrier when an item or path release enables another role. A bounded scope can cover subsequent work without per-item approval. Workers may propose extensions while waiting; informational acknowledgements and unchanged idle notices should not displace useful work.

Sending confirms durable inbox storage, not agreement. Reply clearly when accepting, declining or deferring work; describe the scope you undertake. Acceptance may share a progress report when no earlier confirmation is needed. Changes to commitments, handoff readiness, completion and failure should reach affected parties. Informational messages need no automatic reply. Sender chooses recipients; the harness does not infer relevance from names mentioned in text.

Reference the accepted proposal by reply_to rather than restating unchanged parameters. A changed parameter is a new proposal: identify what it replaces and obtain affected acceptance before using it. Merge newer inbox facts before sending a missing-information reminder; allow for messages crossing while a teammate is deciding. Reaffirm only when it changes execution, not to answer every crossed status update.

Accepted scopes remain valid through their agreed completion/release conditions. A progress observation is evidence, not a new approval gate. If changing an active commitment, identify the replacement scope and obtain the affected executor's acknowledgement before assuming it has stopped or changed course. A planned finite segment may still be running when a request arrives.

A pause request uses the same addressed message format. The executor reviews it at the next selected message boundary, resolves the affected in-flight action by completion or its own stop tool, then reports actual execution, payload and remaining commitments. Receipt alone is not paused confirmation; absence of an action is not geometric path clearance. Pause retains the current role/scope and ownership; return_task closes an adaptive delegation. Continue unaffected authorized work where supported. The current stop tool acts only on an owned action; Controller cannot directly stop a delegate's action. This cooperative protocol has no immediate-preemption guarantee: blocking tools, finite sequences and model processing can delay handling. Choose boundaries with the expected need for plan changes in mind.

Keep changed conditions, remaining constraints, needed decisions and concise evidence. Unchanged status confirmations do not create work. Pending replies block only dependent work; independently authorized work can continue.

Share a run-local lesson when it changes another role's choices, with evidence and applicability limits or an allowed artifact reference. Hierarchical Workers share through Coordinator, peers with affected Workers, and adaptive delegates through Controller; routine trials need no separate announcement. A lesson is neither a new authorization nor verified object truth.

Messages and system events share the configured batch byte/count budget; remaining_messages and remaining_events identify items still queued. Oversized notices require a shorter message or an artifact reference. Delivery batches have IDs. The client acknowledges receipt on its next call; this is not business acceptance. Until that receipt, the batch may be redelivered with the same message IDs. Do not repeat accepted physical work because its message appears again. `history` retrieves your own conversation without creating commitments.

A Worker explicitly declaring available produces a factual system event for Coordinator. It means the Worker reports availability, not that an evaluator verified its goal. Tool completion and native session idle do not imply availability.

Messages are batched at tool-return boundaries for both Workers and Coordinator. New messages do not cancel active tools. When the native session is idle, pending mail, an agent-selected deadline, or saved continuing work may trigger a continuation, including during a normal nonblocking action. The arm remains busy; blocking robot actions and uncertain actions still require their existing outcome. There is one pending/in-flight notification per recipient; uncertain notification outcomes are checked rather than blindly resent. No running-turn steer is used by this protocol.

Message traffic and natural-language misunderstandings remain part of agent behavior; no hidden planner fixes them. Cancellation text is a coordination request, not emergency stop.

// Thin local adapter for a single Agent-selected operation. No task planner,
// motion queue, object perception or native-session management.
function validateRoleClient({tools, python, connection, text}) {
  if (typeof python!=='string' || !python.trim() || typeof connection!=='string' ||
      !connection.trim() || python===connection)
    throw new Error('Provide distinct nonempty Python executable and connection path');
  if (typeof tools?.exec_command!=='function' || typeof text!=='function')
    throw new Error('Role tools need callable tools.exec_command and text');
}
function compactFeedback(result) {
  const value = {...result}, observation = result.observation;
  if (observation?.robot) {
    value.observation = {...observation, feedback_detail:'compact', robot:{}};
    for (const [name, state] of Object.entries(observation.robot)) {
      const robot = {...state};
      if (Array.isArray(robot.joint_rad) && robot.joint_rad.every(Number.isFinite))
        delete robot.joint_rad;
      if (Array.isArray(robot.joint_velocity_rad_s) && robot.joint_velocity_rad_s.length &&
          robot.joint_velocity_rad_s.every(Number.isFinite)) {
        robot.max_abs_joint_velocity_rad_s = Math.max(...robot.joint_velocity_rad_s.map(Math.abs));
        delete robot.joint_velocity_rad_s;
      }
      value.observation.robot[name] = robot;
    }
  }
  if (result.original_result) value.original_result = compactFeedback(result.original_result);
  return value;
}
async function robotTool({tools, python, connection, op, data = {}, callId,
                          views = [], detail = 'compact', text, image}) {
  validateRoleClient({tools,python,connection,text});
  if (!['compact','full'].includes(detail)) throw new Error('detail must be compact or full');
  if (!Array.isArray(views) || views.some(v => typeof v !== 'string') || new Set(views).size !== views.length)
    throw new Error('Choose distinct camera names explicitly');
  if (['move','gripper','action_batch'].includes(op) && data.cameras === undefined)
    data = {...data, cameras:views};
  const quote = value => "'" + String(value).replaceAll("'", "'\\''") + "'";
  const args = [python, '-m', 'robofrontier.pipeline_client', '--connection', connection,
                '--quiet', ...(callId ? ['--call-id', callId] : []), op, '--data', JSON.stringify(data)];
  let reply = await tools.exec_command({cmd:args.map(quote).join(' '), yield_time_ms:1000, max_output_tokens:12000});
  let output = reply.output;
  while (reply.session_id) {
    reply = await tools.write_stdin({session_id:reply.session_id, chars:'', yield_time_ms:30000, max_output_tokens:12000});
    output += reply.output;
  }
  let result;
  try { result = JSON.parse(output); }
  catch { throw new Error('Incomplete client JSON; inspect local pending requests before retrying any physical action'); }
  // Load only images already returned by this operation. Never silently issue
  // another observation, change camera selection or hide the action result.
  const loaded = [], errors = [];
  const observation = result.observation ?? (op === 'job' ? result.original_result?.observation : undefined);
  const selections = await Promise.allSettled(views.map(async camera => {
    const path = observation?.images?.[camera];
    if (!path) throw new Error('Selected camera is not in this result');
    return await tools.view_image({path});
  }));
  for (let i=0;i<views.length;i++) {
    const selection=selections[i], camera=views[i];
    if (selection.status==='fulfilled') loaded.push({camera,value:selection.value});
    else errors.push({camera,error:String(selection.reason)});
  }
  if (errors.length) result = {...result, view_errors:errors};
  // Presentation only: every caller still receives the original full result.
  text(detail==='full' ? result : compactFeedback(result));
  for (const {camera,value} of loaded) {
    text({camera,captured_at:observation.captured_at,sim_time_s:observation.sim_time_s});image(value.image_url);
  }
  return result;
}
// A non-robot operation still ends a role decision boundary. Keep the selected
// operation/output intact, then deliver addressed events without steering it.
async function roleStep({operation, ...base}) {
  validateRoleClient(base);
  if (typeof operation!=='function') throw new Error('roleStep needs an operation');
  let value, failure, failed=false;
  try { value=await operation(); }
  catch (error) {failed=true;failure=error;}
  try {
    await robotTool({...base,op:'inbox',data:{},views:[],callId:undefined,text:events=>{
      if (events.error || events.run_paused || events.run_ended || events.messages?.length ||
          events.system_events?.length || events.remaining_messages || events.remaining_events)
        base.text({role_events:events});
    }});
  } catch (error) {
    base.text({role_event_delivery_error:String(error),note:'Selected work was not replayed. Restore own inbox before relying on old coordination facts.'});
  }
  if (failed) throw failure;
  return value;
}
// An explicit finite chunk, authored by the Agent. It cannot select objects,
// interpret images, wait on teammate conditions or invent another command.
async function robotSequence({steps, views = [], messageBoundary = 'step', ...base}) {
  if (!['step','sequence'].includes(messageBoundary))
    throw new Error('messageBoundary must be step or sequence');
  if (!Array.isArray(steps) || !steps.length)
    throw new Error('Provide a nonempty finite list of decided actions');
  const ids = new Set();
  for (const step of steps) {
    if (!['move','gripper'].includes(step.op)) throw new Error('Sequences allow move and gripper only');
    if (typeof step.callId !== 'string' || !/^[A-Za-z0-9_-]{1,128}$/.test(step.callId) || ids.has(step.callId))
      throw new Error('Give every step a distinct stable callId');
    ids.add(step.callId);
    if (step.views !== undefined && !Array.isArray(step.views)) throw new Error('Step views must be explicit camera names');
  }
  const results = [];
  let completed = 0, reason = 'complete';
  for (let i=0;i<steps.length;i++) {
    const step = steps[i], selected = step.views ?? (i===steps.length-1 ? views : []);
    const result = await robotTool({...base, op:step.op, data:step.data ?? {}, callId:step.callId, views:selected});
    results.push(result);
    const status = result.execution?.status;
    if (status==='completed') completed++;
    if (result.run_paused || result.wait_status==='paused') reason='paused';
    else if (result.run_ended) reason='ended';
    else if (['running','pending','uncertain'].includes(result.execution_status) || ['running','uncertain'].includes(status)) reason='execution_pending';
    else if (result.error) reason='error';
    else if (status==='failed') reason='failed';
    else if (status!=='completed') reason='unconfirmed_result';
    else if (result.system_events?.length || (messageBoundary==='step' && result.messages?.length)) reason='events';
    else if (result.view_errors?.length) reason='view_error';
    else if (selected.length && i<steps.length-1) reason='observation_boundary';
    if (reason!=='complete') break;
  }
  const value = {sequence:{completed_steps:completed,total_steps:steps.length,next_step:results.length,stop_reason:reason,message_boundary:messageBoundary},results};
  base.text({sequence:value.sequence});
  return value;
}
if (typeof module !== 'undefined') module.exports = {robotTool,robotSequence,roleStep};

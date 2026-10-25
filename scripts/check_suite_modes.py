"""Freeze/load the four topologies without starting a model or dispatcher."""
import json,tempfile
from pathlib import Path
from robofrontier.suite.scenes import ROOT,catalog,case_directory
from robofrontier.run_setup import prepare_run,verify_inputs,write_json
from robofrontier.pipeline import World
from robofrontier.task_config import load_harness

def check():
    representatives=('transport-a12-d3','relay-a16-d3','mosaic-a08-7x21-d3','dial-n08-d3');results=[]
    with tempfile.TemporaryDirectory() as temp:
        for case_id in representatives:
            case=next(c for c in catalog()['cases'] if c['id']==case_id)
            for mode in ('centralized','hierarchical','peer','adaptive'):
                cfg=load_harness();cfg['topology']=mode;config=Path(temp)/'harness.json';write_json(config,cfg)
                run=prepare_run(Path(temp)/(case_id+'-'+mode),case_directory(case)/'task.json',config)
                world=None
                try:
                    errors=verify_inputs(run)
                    if errors:raise RuntimeError('Frozen input mismatch: '+str(errors))
                    private=json.loads((run/'private/inputs/task.json').read_text());public=json.loads((run/'public/inputs/task.json').read_text())
                    if any('initial_position_m' in o for o in public.get('objects',[])):raise RuntimeError('Policy initial-pose leak')
                    world=World(run,private);world.start_trial()
                    for _ in range(5):world.step()
                    facts=world.robot_facts(cfg);world.end_trial()
                    results.append(dict(case_id=case_id,topology=mode,passed=True,robots=len(world.arms),robot_facts_bytes=len(json.dumps(facts).encode()),llm_started=False))
                except Exception as error:results.append(dict(case_id=case_id,topology=mode,passed=False,error=type(error).__name__+': '+str(error),llm_started=False))
                finally:
                    if world is not None:world.close()
                print(json.dumps(results[-1]),flush=True)
    write_json(ROOT/'runs/oracle/suite-mode-smoke.json',dict(scope='Engineering freeze/load only, not policy or task completion',results=results))
    if not all(r['passed'] for r in results):raise SystemExit('A mode failed to freeze/load')

if __name__=='__main__':check()

"""Submit one benchmark batch without an LLM supervisor (macOS or Linux CPU)."""
import argparse
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
from datetime import datetime

from robofrontier.task_families import ROOT
from scripts.benchmark import load_env, prepare_batch, run_batch
from scripts.luna_cpu import batch_codex_home, render_probe, sandbox_probe

DEFAULT_CASES = ['transport-a04-d1', 'transport-a08-d1', 'mosaic-a04-6x12-d1',
                 'mosaic-a06-9x18-d1', 'relay-a08-d1', 'dial-best5-n04-d1']


def execution_environment(system, base):
    if system == 'Darwin':
        env = dict(base)
        # Native GLFW/OpenGL context; this does not promise CPU rendering.
        env.pop('MUJOCO_GL', None)
        env.pop('PYOPENGL_PLATFORM', None)
        return env
    if system == 'Linux':
        env = dict(base)
        env.update(MUJOCO_GL='osmesa', PYOPENGL_PLATFORM='osmesa',
                   LIBGL_ALWAYS_SOFTWARE='true', GALLIUM_DRIVER='llvmpipe', CUDA_VISIBLE_DEVICES='')
        return env
    raise ValueError('Supported platforms are macOS and Linux')


def native_render_probe(env):
    code = '''import mujoco
m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>')
d=mujoco.MjData(m)
with mujoco.Renderer(m,64,64) as r:
 r.update_scene(d); frame=r.render(); assert frame.shape == (64,64,3)
print("macOS native MuJoCo offscreen render OK")'''
    return subprocess.run([sys.executable, '-c', code], cwd=ROOT, env=env,
                          check=True, capture_output=True, text=True, timeout=60).stdout.strip()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True,help='New batch directory')
    parser.add_argument('--profile',default='openlux-terra',choices=('openlux-terra','openlux-luna','luna'))
    parser.add_argument('--parallel',type=int,default=6)
    parser.add_argument('--wall-limit-s',type=float,default=300)
    parser.add_argument('--cases',nargs='+',default=DEFAULT_CASES)
    args=parser.parse_args()
    if not 1<=args.parallel<=36:parser.error('parallel must be 1..36')
    import math
    if not math.isfinite(args.wall_limit_s) or args.wall_limit_s<=0:parser.error('wall limit must be positive and finite')
    load_env(ROOT/'.env')
    env=execution_environment(platform.system(),os.environ)
    os.environ.clear();os.environ.update(env)
    key={'openlux-terra':'OPENLUX_API_KEY','openlux-luna':'OPENLUX_API_KEY','luna':'OPENAI_API_KEY'}[args.profile]
    if not os.environ.get(key):raise SystemExit('Set '+key+' in .env or the environment')
    output=args.output.absolute()
    if output.exists():raise SystemExit('Output directory already exists; choose a new batch directory')
    home=batch_codex_home(output)
    home.mkdir(parents=True,exist_ok=True,mode=0o700)
    os.environ['CODEX_HOME']=str(home)
    sandbox_probe(os.environ)
    if platform.system()=='Darwin':print(native_render_probe(os.environ),flush=True)
    else:print('CPU renderer: '+json.dumps(render_probe(os.environ)),flush=True)
    prepare_batch(output,[args.profile],cases=args.cases,wall_limit_s=args.wall_limit_s)
    (output/'codex-home.json').write_text(json.dumps({'codex_home':str(home)},indent=2)+'\n')
    print('Prepared batch: '+str(output),flush=True)
    run_batch(output,args.parallel)
    batch=json.loads((output/'batch.json').read_text())
    print('Batch status: '+batch['status'],flush=True)
    return 0 if batch['status']=='closed' else 1


if __name__=='__main__':raise SystemExit(main())

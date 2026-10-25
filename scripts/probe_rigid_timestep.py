"""Read-only integration-step comparison on passive relay fixtures, no Oracle.

Formal source models and material parameters are unchanged. MuJoCo's default
refsafety clamp remains enabled: at5ms,4ms contact time constants are internally
bounded to at least10ms. This is the effect of changing dt alone, not a test
with identical effective contact stiffness across the candidates.
"""
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from robofrontier.suite.v2 import ROOT,cases,directory
from robofrontier.run_setup import write_json
from scripts.check_rigid_relay import hanging_fixture


def main():
    fixtures=[]
    for name,level,ident,perturbation in (
        ('L2_closed_rings',2,'payload_01',0.),
        ('L3_in_plane',3,'payload_01',.15),
        ('L3_out_of_plane',3,'payload_02',.15),
    ):
        case=next(c for c in cases() if c['id']==f'relay-a08-d{level}')
        folder=directory(case);source=folder/'scene.xml'
        model=mujoco.MjModel.from_xml_path(str(source))
        refsafety=not bool(model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_REFSAFE))
        rows=[]
        for dt in (.001,.002,.005):
            trials=[hanging_fixture(case,dt,payload_id=ident,initial_hinge_rad=perturbation) for _ in range(3)]
            row=trials[0]
            row['wall_samples_s']=[t['wall_s'] for t in trials]
            row['median_wall_s']=float(np.median(row['wall_samples_s']))
            row['all_repeats_retained']=all(t['retained'] for t in trials)
            row['all_repeats_numerically_clean']=all(t['numerical_warnings']==0 for t in trials)
            rows.append(row)
        fixtures.append(dict(name=name,source=str(source.relative_to(ROOT)),
            source_scene_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            refsafety_enabled=refsafety,declared_ring_contact_time_constant_ms=4.,
            candidate_5ms_minimum_contact_time_constant_ms=10.,trials=rows))
        print(json.dumps(dict(fixture=name,penetration_mm_by_timestep={r['timestep_ms']:r['max_ring_contact_penetration_mm'] for r in rows})),flush=True)
    report=dict(scope='Three isolated passive suspension fixtures,5 simulated seconds each,3 repetitions per timestep. No robot actions, no full-task Oracle, no rendering. Same declared geometry, masses, material and initial states per fixture; only in-memory dt differs. Enabled refsafety alters effective contact softness at5ms.',
        formal_parameters_changed=False,oracle_started=False,policy_started=False,
        decision='Keep formal2ms;5ms increases contact penetration substantially. This counterexample rejects a global change, but does not establish5ms behaviour for every other task.',
        ring_stock_diameter_mm=5.,fixtures=fixtures,
        official_reference='https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters')
    path=ROOT/'docs/generated/frontier-v2/mechanism-checks/rigid-timestep-comparison.json'
    write_json(path,report);print(path,flush=True)
    return report


if __name__=='__main__':main()

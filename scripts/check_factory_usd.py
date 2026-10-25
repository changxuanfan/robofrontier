"""Run inside Blender's Python: check static USD identities and rigid poses."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from pxr import Usd,UsdGeom


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--usd',type=Path,required=True);p.add_argument('--asset',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(sys.argv[sys.argv.index('--')+1:]);stage=Usd.Stage.Open(str(a.usd.resolve()));source=json.loads((a.asset/'scene.json').read_text())
    expected={g['id']:g for g in source['geoms']};cache=UsdGeom.XformCache();seen=set();max_position=0.;max_rotation=0.;errors=[]
    for prim in stage.Traverse():
        attr=prim.GetAttribute('userProperties:source_geom_id')
        if not attr:continue
        gid=int(attr.Get())
        if gid in seen:errors.append('Duplicate source geom ID '+str(gid));continue
        seen.add(gid)
        if gid not in expected:errors.append('Unknown source geom ID '+str(gid));continue
        g=expected[gid];transform=np.asarray(cache.GetLocalToWorldTransform(prim),dtype=float)
        position=transform[3,:3];rotation=transform[:3,:3].T
        lengths=np.linalg.norm(rotation,axis=0)
        if np.any(lengths<=0):errors.append('Degenerate visual scale '+str(gid));continue
        rotation/=lengths
        max_position=max(max_position,float(np.max(abs(position-np.array(g['position'])))))
        max_rotation=max(max_rotation,float(np.max(abs(rotation-np.array(g['rotation'])))))
        name=prim.GetAttribute('userProperties:source_geom_name')
        if not name or name.Get()!=g['name']:errors.append('Source name mismatch '+str(gid))
    if seen!=set(expected):errors.append('Missing or extra source geom identities')
    if max_position>1e-5 or max_rotation>1e-6:errors.append('Visual poses exceed declared float32 conversion tolerance')
    if UsdGeom.GetStageUpAxis(stage)!='Z' or abs(UsdGeom.GetStageMetersPerUnit(stage)-1)>1e-12:errors.append('USD axis or metre scale differs')
    result=dict(passed=not errors,errors=errors,source_scene_sha256=source['source_scene_sha256'],visual_geom_ids_checked=len(seen),max_position_error_m=max_position,max_rotation_component_error=max_rotation,
                scope='Static USD source IDs, names, axis, metre scale and visual poses; no Isaac dynamics or animation acceptance')
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    if errors:raise RuntimeError('USD visual conversion audit failed')


if __name__=='__main__':main()

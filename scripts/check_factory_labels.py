"""Ray-test factory wayfinding plaques in the roof-hidden top view, without physics."""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from robofrontier.factory_labels import region_title


def check(scene):
    scene=Path(scene);manifest=json.loads((scene.parent/'manifest.json').read_text())
    model=mujoco.MjModel.from_xml_path(str(scene));data=mujoco.MjData(model)
    mujoco.mj_kinematics(model,data)
    mask=np.ones(6,dtype=np.uint8);mask[1]=0
    rows=[]
    for label in manifest['factory_labels']['labels']:
        x,y,_=label['position_m'];w,h=label['half_size_m'];hits=[]
        for dx in (-.95,0.,.95):
            for dy in (-.90,0.,.90):
                target=np.array([x+dx*w,y+dy*h,32.2]);gid=np.array([-1],dtype=np.int32)
                distance=mujoco.mj_ray(model,data,target,np.array([0.,0.,-1.]),mask,True,-1,gid)
                name=model.geom(int(gid[0])).name if gid[0]>=0 else None
                hits.append(dict(sample_xy_m=target[:2].tolist(),first_hit=name,passed=name==label['id']))
        centered=True
        if label['kind']=='region':
            region=next(r for r in manifest['regions']if label['id']=='ws_region_'+r['id'])
            expected,_=region_title(region)
            centered=bool(np.allclose(data.geom_xpos[model.geom(label['id']).id],expected,rtol=0,atol=1e-8))
        rows.append(dict(id=label['id'],kind=label['kind'],top_centered=centered if label['kind']=='region' else None,
                         passed=centered and all(h['passed']for h in hits),rays=hits))
    return dict(passed=all(r['passed']for r in rows),labels=len(rows),ray_samples=sum(len(r['rays'])for r in rows),
                physics_steps=0,scope='Initial-state top visibility, roof group excluded; does not claim visibility from every oblique camera or future pose',rows=rows)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--scene',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=check(args.scene);args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items()if k!='rows'}))
    raise SystemExit(0 if result['passed']else 1)

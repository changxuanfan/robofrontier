"""Isolated one-variable contact/control probes; never official suite scores."""
import argparse,json,math
from pathlib import Path
import numpy as np
from robofrontier.suite.oracle import Trial
from robofrontier.suite.scenes import ROOT,rz
from robofrontier.run_setup import write_json

def run(out,hold=15.):
    out=Path(out);out.mkdir(parents=True,exist_ok=False);summaries=[]
    scenarios=[('baseline_cart',{}),('aperture',{'force':None}),('force30',{'force':30.}),('stiff_contact',{'solref':.004}),('noslip0',{'noslip':0}),('dt001',{'dt':.001})]
    for label,params in scenarios:
        t=Trial(ROOT/'configs/environments/cube_transport/suite-transport-a04-d2',out/label,frames=False);report=dict(scope='Counterfactual calibration only',passed=False,parameters=params)
        try:
            if 'solref' in params:
                for g in range(t.model.ngeom):
                    body=t.model.body(t.model.geom_bodyid[g]).name or ''
                    if body=='A_item1' or body.startswith(('A_left_finger','A_right_finger')):
                        t.model.geom_priority[g]=1;t.model.geom_solref[g]=[params['solref'],1];t.model.geom_solimp[g]=[.95,.99,.001,.5,2]
            if 'noslip' in params:t.model.opt.noslip_iterations=params['noslip']
            if 'dt' in params:t.model.opt.timestep=params['dt'];t.stride=round(.02/params['dt'])
            t.wait(.5);t.pick('A','A_item1',force=params.get('force',50.));arm=t.arms['A'];obj=t.data.body('A_item1');orot=obj.xmat.reshape(3,3).copy();offset=orot.T@(t.data.site_xpos[arm.site]-obj.xpos);relative=orot.T@t.data.site_xmat[arm.site].reshape(3,3)
            r=rz(math.pi/2);target=np.array([.105,-.23,.68])+r@offset;rr=r@relative
            t.execute('calibration-carry',{'A':t.pose_command('A',target,rr,.07,.35)});t.wait(hold)
            after=t.data.body('A_item1');drift=np.linalg.norm(after.xpos-(np.array([.105,-.23,.68])));report.update(passed=bool(t.finger_contacts('A','A_item1') and drift<.01),carry_object_error_m=float(drift),bilateral_contact=t.finger_contacts('A','A_item1'),width_m=arm.observe()['gripper_width_m']);t.snapshot('final')
        except Exception as e:report['error']=str(e)
        finally:t.close(report);summaries.append(dict(label=label,**report));print(json.dumps(summaries[-1]),flush=True)
    write_json(out/'summary.json',summaries)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--hold',type=float,default=15.);a=p.parse_args();run(a.output,a.hold)

"""Reusable shot planning over station placements; independent of source clocks."""
import math
import numpy as np


def place_station_camera(manifest, case_id, camera):
    """Rigidly place a frozen task camera in its factory station, without scaling."""
    place=next(p for p in manifest['placements'] if p['case_id']==case_id)
    yaw=place['yaw_rad'];c,s=math.cos(yaw),math.sin(yaw)
    rotation=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
    shift=np.asarray(place['translation_m'])
    return dict(camera,id=case_id+'/front',case_id=case_id,
        source_camera=camera['id'],position_m=(rotation@np.asarray(camera['position_m'])+shift).tolist(),
        target_m=(rotation@np.asarray(camera['target_m'])+shift).tolist())


def shots(manifest, seconds=6.):
    result = [dict(id='overview', duration_s=seconds, position_m=[0,-32,31],
                   end_position_m=[4,-29,29], target_m=[0,0,.8], vertical_fov_deg=55,
                   hidden_layers=['roof']),
              dict(id='corner', duration_s=seconds, position_m=[-23,-20,6.5],
                   end_position_m=[-19,-22,7.5], target_m=[-7,-5,1.2], vertical_fov_deg=65,
                   hidden_layers=['roof'])]
    # Each family progresses L1 -> L2 -> L3; size is the secondary order.
    for family in ('transport','mosaic','relay','dial'):
        places = [p for p in manifest['placements'] if p['family']==family]
        def order(p):
            return (int(p['case_id'].rsplit('-d',1)[1]), p['case_id'])
        for p in sorted(places,key=order):
            x0,y0,x1,y1=p['planning_bounds_xy_m'];x,y=(x0+x1)/2,(y0+y1)/2
            span=max(x1-x0,y1-y0)
            result.append(dict(id=p['case_id'], duration_s=seconds,
                position_m=[x,y-span*.7,span*1.05+2],
                end_position_m=[x+span*.12,y-span*.6,span*1.02+2],
                target_m=[x,y,.8],vertical_fov_deg=52,hidden_layers=['roof']))
    return result


def camera_at(plan, seconds):
    if not math.isfinite(seconds) or seconds<0:raise ValueError('Invalid camera clock')
    duration=sum(s['duration_s'] for s in plan)
    clock=seconds%duration
    for shot in plan:
        if clock<shot['duration_s']:break
        clock-=shot['duration_s']
    fraction=clock/shot['duration_s'];fraction=fraction*fraction*(3-2*fraction)
    start=np.asarray(shot['position_m']);end=np.asarray(shot.get('end_position_m',start))
    return dict(position_m=((1-fraction)*start+fraction*end).tolist(),
                target_m=shot['target_m'],vertical_fov_deg=shot['vertical_fov_deg'],
                projection='perspective',hidden_layers=shot['hidden_layers'])

"""Reserve real floor space for factory wayfinding, outside equipment footprints."""


def bounds(center, half):
    x,y=center[:2];w,h=half
    return [x-w,y-h,x+w,y+h]


def overlaps(a,b,margin=0.):
    return a[0]<b[2]+margin and a[2]>b[0]-margin and a[1]<b[3]+margin and a[3]>b[1]-margin


def region_title(region):
    x0,y0,x1,y1=region['bounds_xy_m']
    return [(x0+x1)/2,y1-.6,.004],[3.6,.5]


def place_label(candidates,half,occupied,margin=.10):
    for position in candidates:
        box=bounds(position,half)
        if not any(overlaps(box,other,margin)for other in occupied):
            return list(position),box
    raise ValueError('No clear floor space for label; adjust layout instead of hiding text under furniture')

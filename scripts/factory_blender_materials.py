"""Explicit presentation materials; never mutate frozen MuJoCo assets or RGBA events."""
import re


ROBOT_BODY = re.compile(r'__[A-Z]_(?:link\d|hand|left_?finger|right_?finger)$')


def robot_role(spec):
    """Classify the neutral authored robot parts, leaving colored indicators intact."""
    rgb = spec['rgba'][:3]
    if max(rgb) - min(rgb) < .06:
        if max(rgb) <= .30:
            return 'joint'
        if min(rgb) >= .75:
            return 'shell'
    return 'hardware'


def task_role(geom):
    name=geom.get('name','')
    if '__' not in name:return None
    if '_label' in name or '_print' in name:return 'print'
    if (name.startswith('relay-') and re.search(r'__payload_\d+_beam$',name)
            or name.startswith('mosaic-') and re.search(r'__[A-Z]_stock\d+_geom$',name)):
        return 'painted_wood'
    return None


def resolve(spec, role=None, calibrated=True):
    result = dict(spec)
    # Replay events arrive as NumPy float32; normalize at this boundary so the
    # effective material audit remains JSON-safe when a new color appears.
    result['rgba']=[float(c) for c in result['rgba']]
    # Negative MuJoCo values are unspecified, not physical PBR coefficients.
    for key, default in [('roughness', .52), ('metallic', 0.)]:
        value = result.get(key, -1.)
        result[key] = float(default if value < 0 else min(1., value))
    if calibrated and role in ('joint', 'shell'):
        # A documented display palette, not an assumed global sRGB conversion.
        result.update(rgba=[.025, .030, .036, result['rgba'][3]] if role == 'joint'
                      else [.67, .70, .72, result['rgba'][3]],
                      roughness=.54 if role == 'joint' else .46,
                      metallic=0., specular=.30)
    if role in ('print','painted_wood'):
        result.update(roughness=.80 if role=='print' else .58,metallic=0.,specular=.18 if role=='print' else .25)
    return result

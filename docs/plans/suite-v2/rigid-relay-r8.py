"""Cable-free relay apparatus: contact-linked eyes and a passive spring hinge.

The closed eyes are hollow rings built from colliding capsules, not a weld or
an equality attachment. The weight is an independent free body in both levels.
Only the manufactured L3 beam hinge has a joint. No apparatus has an actuator.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

from .scenes import box, cylinder, vector


PROFILE = dict(
    weight_mass_kg=.600,
    weight_material='Steel calibration-weight form; nominal manufactured mass, not a measured specimen',
    hook_centerline_radius_m=.022,
    eye_centerline_radius_m=.018,
    ring_stock_radius_m=.0025,
    ring_segments=20,
    pendulum_shaft_length_m=.120,
    pendulum_shaft_radius_m=.005,
    beam_hinge_stiffness_nm_rad=18.,
    beam_hinge_damping_nms_rad=.55,
    beam_hinge_limit_rad=math.radians(25),
    timestep_s=.002,
    contact_time_constant_s=.004,
    material_calibration_verified=False,
)
METAL = (.28, .34, .39, 1)
BRASS = (.73, .49, .19, 1)


def capsule(body, name, a, b, radius, mass, color=METAL):
    return ET.SubElement(body, 'geom', name=name, type='capsule',
                         fromto=vector([*a, *b]), size=str(radius), mass=str(mass),
                         rgba=vector(color), friction='.5 .002 .0001', condim='3',
                         solref='.004 1', solimp='.99 .999 .001 .5 2')


def ring(body, name, center, radius, plane):
    center = np.asarray(center)
    theta = np.arange(PROFILE['ring_segments']+1)*2*math.pi/PROFILE['ring_segments']
    points = np.zeros((len(theta), 3))
    points[:, plane[0]] = radius*np.cos(theta)
    points[:, plane[1]] = radius*np.sin(theta)
    points += center
    # Stainless-steel stock, with every capsule contributing physical inertia.
    mass = 7800*math.pi*PROFILE['ring_stock_radius_m']**2*2*math.pi*radius
    for i, (a, b) in enumerate(zip(points, points[1:])):
        capsule(body, f'{name}_{i:02d}', a, b, PROFILE['ring_stock_radius_m'],
                mass/PROFILE['ring_segments'])
    return mass


def spring_hinge(root, beam, rise):
    # The pin sits above the beam centre so its housing/coil clears the table
    # in the neutral, tabletop-supported pose.
    pivot = np.array([rise, 0., .014])
    upper = ET.SubElement(beam, 'body', name='payload_distal', pos=vector(pivot))
    limit = PROFILE['beam_hinge_limit_rad']
    ET.SubElement(upper, 'joint', name='payload_flexion', type='hinge', axis='1 0 0',
                  stiffness=str(PROFILE['beam_hinge_stiffness_nm_rad']),
                  damping=str(PROFILE['beam_hinge_damping_nms_rad']),
                  springref='0', limited='true', range=vector([-limit, limit]))
    segment = beam.find("geom[@name='payload_beam_segment1']")
    if segment is None:
        raise ValueError('Expected two-segment elbow before adding the hinge')
    beam.remove(segment)
    segment.set('pos', vector(-pivot))
    upper.append(segment)
    # A visible pin housing and spring coil explain the actual passive joint.
    cylinder(beam, 'payload_hinge_housing', pivot, [.030, .032], BRASS, mass=.08,
             quat=vector([math.sqrt(.5), 0, math.sqrt(.5), 0]))
    points = [pivot+np.array([-.026+.052*t/(6*math.pi),
                              .033*math.cos(t), .033*math.sin(t)])
              for t in np.linspace(0, 6*math.pi, 61)]
    for i, (a, b) in enumerate(zip(points, points[1:])):
        g = capsule(beam, f'payload_hinge_coil_{i}', a, b, .0015, 0., METAL)
        g.set('contype', '0'); g.set('conaffinity', '0')
    return upper, pivot


def weight(root, folder, eye_world, rotation, level):
    from .scenes import texture
    from ..robot_control import matrix_quat
    body = ET.SubElement(root.find('worldbody'), 'body', name='weight',
                         pos=vector(eye_world), quat=vector(matrix_quat(rotation)))
    ET.SubElement(body, 'freejoint', name='weight_free')
    ring_mass = ring(body, 'weight_eye', [0, 0, 0], PROFILE['eye_centerline_radius_m'], (1, 2))
    # At pickup the weight stands on the table. L3's hard shaft is initially
    # almost horizontal; it can swing down naturally when the beam is lifted.
    center = np.array([-.120 if level == 3 else 0., 0., -.055])
    bar_mass = .035 if level == 3 else .010
    capsule(body, 'weight_shaft', [0, 0, -.018], center+[0, 0, .030],
            PROFILE['pendulum_shaft_radius_m'], bar_mass)
    target_mass = PROFILE['weight_mass_kg']
    cylinder(body, 'weight_base', center+[0, 0, -.021], [.033, .004], METAL,
             mass=target_mass*.20, solref='.004 1')
    cylinder(body, 'weight_core', center+[0, 0, -.003], [.029, .014], METAL,
             mass=target_mass*.65, solref='.004 1')
    # Truncated-cone shoulders are a convex solid, leaving a recognisable
    # scale-weight silhouette rather than an anonymous cube or cylinder.
    vertices = [[radius*math.cos(t)+center[0], radius*math.sin(t), z+center[2]]
                for radius, z in ((.029, .011), (.013, .024))
                for t in np.arange(24)*2*math.pi/24]
    ET.SubElement(root.find('asset'), 'mesh', name='weight_shoulder_mesh', vertex=vector(np.ravel(vertices)))
    ET.SubElement(body, 'geom', name='weight_shoulder', type='mesh', mesh='weight_shoulder_mesh',
                  mass=str(target_mass*.15), rgba=vector(METAL), solref='.004 1')
    marking = texture(root, folder, 'weight_mass_mark', '600 g', size=(512,256), font_size=130)
    box(body, 'weight_mass_label', center+[0, -.0292, -.001], [.022, .00015, .009],
        visual=True, material=marking)
    return body, center, target_mass+ring_mass+bar_mass


def install(root, beam, beam_world, rotation, mechanism, level, folder):
    """Return updated mechanism and an initial close-camera target."""
    parent = beam; pivot = np.zeros(3)
    if level == 3:
        parent, pivot = spring_hinge(root, beam, mechanism['bend_rise_m'])
        mechanism['kind'] = 'spring_elbow_closed_eye_rigid_pendulum'
    else:
        mechanism['kind'] = 'arc_closed_eye_weight'
    t = math.radians(40.5)
    if level == 2:
        radius = mechanism['radius_m']
        mount = np.array([radius*math.cos(t)-radius*math.cos(math.pi/4), radius*math.sin(t), .020])
        hook = mount-np.array([math.cos(t), math.sin(t), 0])*.090
    else:
        mount = np.array([.1*mechanism['bend_rise_m'], .54, .020])
        normal = np.array([-.6, -mechanism['bend_rise_m'], 0.]); normal /= np.linalg.norm(normal)
        hook = mount+normal*.090
    hook[2] = .0755  # eye height .498 m, beam centre initially .4225 m
    capsule(parent, 'payload_hook_bracket', mount-pivot, hook+[0,0,.022]-pivot, .006, .060)
    ring(parent, 'payload_hook', hook-pivot, PROFILE['hook_centerline_radius_m'], (0,2))
    # Perpendicular closed rings, linked with positive initial surface clearance.
    # Their connection is maintained by ordinary collision forces, not a weld.
    eye_local = hook+[-.014,0,-.018]
    eye_world = beam_world+rotation@eye_local
    _, center, suspended_mass = weight(root, folder, eye_world, rotation, level)
    root.find('option').set('timestep', str(PROFILE['timestep_s']))
    flag = root.find('option/flag')
    if flag is None: flag = ET.SubElement(root.find('option'), 'flag')
    flag.set('autoreset', 'disable')
    mechanism.update(
        suspension='Two closed, perpendicular steel eyes linked by real contact; no weld or cable',
        weight_mass_kg=PROFILE['weight_mass_kg'],
        suspended_mass_kg=suspended_mass,
        weight_body_free=True, cable_model=None, cable_count=0,
        beam_bodies=['payload', 'payload_distal'] if level == 3 else ['payload'],
        tray=None, rigid_suspension_profile=dict(PROFILE),
        initial_condition='Beam and weight are table-supported; closed eyes start interlinked, with clearance',
        pendulum=dict(shaft_length_m=PROFILE['pendulum_shaft_length_m'],
                      shaft_radius_m=PROFILE['pendulum_shaft_radius_m'],
                      connection='Free-body contact pivot through closed eyes') if level == 3 else None,
        elastic_hinge=dict(axis='Beam-local X; out-of-plane bending', rest_angle_rad=0.,
                           stiffness_nm_rad=PROFILE['beam_hinge_stiffness_nm_rad'],
                           damping_nms_rad=PROFILE['beam_hinge_damping_nms_rad'],
                           limit_rad=PROFILE['beam_hinge_limit_rad']) if level == 3 else None,
    )
    return mechanism, eye_world+rotation@center

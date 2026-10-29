"""Shared Snowbot physics helpers, independent of Isaac and ROS."""
import numpy as np

class MeasuredJointVelocity:
    """Velocity of measured encoder positions at their acquisition timestamps.

    PhysX instantaneous DOF velocity can disagree with stabilized position
    increments. No commands or planned positions enter this measurement model.
    Raw native velocity remains in the experiment archive for comparison.
    """
    def __init__(self):self.previous=None

    def update(self,stamp,positions,initial_velocity):
        positions=np.asarray(positions,dtype=float)
        if not np.isfinite(np.r_[stamp,positions]).all():raise ValueError('ENCODER_NONFINITE')
        if self.previous is None:velocity=np.asarray(initial_velocity,dtype=float)
        else:
            previous_stamp,previous=self.previous;dt=stamp-previous_stamp
            if not 0 < dt <= .1 or positions.shape!=previous.shape:raise ValueError('ENCODER_TIME_OR_LAYOUT')
            velocity=(positions-previous)/dt
        if velocity.shape!=positions.shape or not np.isfinite(velocity).all():raise ValueError('ENCODER_VELOCITY')
        self.previous=(stamp,positions.copy())
        return velocity


def joint_drive_parameters(names, limits, cfg):
    """Optional per-joint simulation calibration; never hardware drive gains.

    Keep the original global defaults and URDF effort ceilings. Overrides let
    identical reference replays use different arm/waist damping assumptions.
    """
    overrides = cfg.get('joint_drive_overrides', {})
    if not isinstance(overrides, dict) or set(overrides)-set(names):
        raise ValueError('PHYSICS_DRIVE_JOINTS')
    keys = ('stiffness_nm_rad', 'damping_nm_s_rad', 'max_effort_nm')
    kp, kd, effort = [], [], []
    for name in names:
        entry = overrides.get(name, {})
        if not isinstance(entry, dict) or set(entry)-set(keys):
            raise ValueError('PHYSICS_DRIVE_FIELDS:'+name)
        ceiling = float(limits[name].get('effort'))
        values = [float(entry.get(keys[0], cfg['joint_stiffness_nm_rad'])),
                  float(entry.get(keys[1], cfg['joint_damping_nm_s_rad'])),
                  float(entry.get(keys[2], ceiling))]
        if (not np.isfinite([ceiling, *values]).all() or ceiling <= 0 or
                min(values) <= 0 or values[2] > ceiling):
            raise ValueError('PHYSICS_DRIVE_LIMIT:'+name)
        kp.append(values[0]); kd.append(values[1]); effort.append(values[2])
    return np.array(kp), np.array(kd), np.array(effort)


def wooden_parts(recipe, material):
    kind=recipe.get('geometry_kind','shovel')
    if kind not in ('shovel','rod'):raise ValueError('INVALID_WOOD_GEOMETRY_KIND')
    density = float(material['density_kg_m3'])
    wall = float(material['tray_wall_m'])
    size = np.asarray(recipe['pan_size_m'], dtype=float)
    center = np.asarray(recipe['pan_center_m'], dtype=float)
    radius = float(recipe['shaft_radius_m'])
    ends = np.asarray(recipe['shaft_from_left_grip_m'], dtype=float)
    axis = np.asarray(recipe['shaft_axis_in_tool'], dtype=float)
    left = np.asarray(recipe['left_grip_in_tool_m'], dtype=float)
    if (not np.isfinite(np.r_[density, wall, size, center, radius, ends, axis, left]).all() or
            density <= 0 or radius <= 0 or wall <= 0 or np.any(size <= 2*wall) or
            ends.shape != (2,) or ends[1] <= ends[0] or not np.isclose(np.linalg.norm(axis), 1.)):
        raise ValueError('INVALID_WOOD_GEOMETRY')
    reference = np.eye(3)[np.argmin(abs(axis))]
    transverse = np.cross(reference, axis)
    transverse /= np.linalg.norm(transverse)
    basis = np.column_stack((transverse, np.cross(axis, transverse), axis))
    length = ends[1]-ends[0]
    mass = density*np.pi*radius**2*length
    axial = mass*radius**2/2
    radial = mass*(3*radius**2+length**2)/12
    parts = [dict(name='Shaft', kind='cylinder', radius=radius, length=length,
                  rotation=basis, center=left+axis*np.mean(ends), mass=mass,
                  inertia=basis @ np.diag([radial, radial, axial]) @ basis.T)]
    if kind=='rod':return parts
    def box(name, dims, offset):
        dims = np.asarray(dims)
        m = density*np.prod(dims)
        inertia = m/12*np.diag([dims[1]**2+dims[2]**2,
                                dims[0]**2+dims[2]**2, dims[0]**2+dims[1]**2])
        parts.append(dict(name=name, kind='box', size=dims, center=center+offset,
                          rotation=np.eye(3), mass=m, inertia=inertia))
    x, y, z = size
    box('Bottom', [x, y, wall], np.array([0., 0., -(z-wall)/2]))
    for sign in (-1, 1):
        box('Side'+str(sign).replace('-', 'N'), [x, wall, z-wall],
            np.array([0., sign*(y-wall)/2, wall/2]))
    box('Back', [wall, y-2*wall, z-wall], np.array([-(x-wall)/2, 0., wall/2]))
    return parts


def compound_mass(parts):
    mass = sum(p['mass'] for p in parts)
    com = sum(p['mass']*p['center'] for p in parts)/mass
    inertia = np.zeros((3, 3))
    for p in parts:
        r = p['center']-com
        inertia += p['inertia']+p['mass']*(np.dot(r, r)*np.eye(3)-np.outer(r, r))
    if not np.isfinite(inertia).all() or np.linalg.eigvalsh(inertia).min() <= 0:
        raise ValueError('INVALID_COMPOUND_INERTIA')
    return mass, com, inertia

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



def _urdf_pan_parts(recipe, density):
    """Read closed binary-STL pan components inside the existing CAD envelope."""
    from pathlib import Path
    import xml.etree.ElementTree as ET
    from scipy.spatial.transform import Rotation
    source = Path(recipe['d_handle_urdf'])
    low = np.asarray(recipe['pan_center_m'])-np.asarray(recipe['pan_size_m'])/2
    high = np.asarray(recipe['pan_center_m'])+np.asarray(recipe['pan_size_m'])/2
    parts = []
    record = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    for collision in ET.parse(source).findall('./link/collision'):
        name = collision.get('name', '')
        if not name.startswith('pan_'):
            continue
        mesh, origin = collision.find('geometry/mesh'), collision.find('origin')
        if mesh is None:
            raise ValueError('PAN_REQUIRES_CLOSED_STL_MESH')
        path = Path(mesh.get('filename', ''))
        if not path.is_absolute():
            path = source.parent/path
        payload = path.read_bytes()
        count = int.from_bytes(payload[80:84], 'little')
        if len(payload) != 84+50*count or count < 4:
            raise ValueError('INVALID_PAN_STL')
        triangles = np.frombuffer(payload, dtype=record, count=count, offset=84)['vertices'].astype(float)
        scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
        if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale <= 0):
            raise ValueError('INVALID_PAN_MESH_SCALE')
        triangles *= scale
        vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
        faces = inverse.reshape(-1, 3)
        edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
        _, edge_ids, counts = np.unique(np.sort(edges, axis=1), axis=0, return_inverse=True, return_counts=True)
        orientation = np.bincount(edge_ids, weights=np.where(edges[:, 0] < edges[:, 1], 1., -1.))
        a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
        cross = np.cross(b-a, c-a)
        if (not np.isfinite(triangles).all() or np.any(np.linalg.norm(cross, axis=1) < 1e-12) or
                np.any(counts != 2) or np.any(orientation != 0)):
            raise ValueError('PAN_MESH_NOT_CLOSED_OR_ORIENTED')
        volumes = np.einsum('ni,ni->n', a, np.cross(b, c))/6
        volume = volumes.sum()
        if volume <= 0:
            raise ValueError('PAN_MESH_VOLUME')
        summed = a+b+c
        com = np.einsum('n,ni->i', volumes, summed)/(4*volume)
        second = density/20*(np.einsum('n,ni,nj->ij', volumes, summed, summed)+
            sum(np.einsum('n,ni,nj->ij', volumes, v, v) for v in (a, b, c)))
        mass = density*volume
        inertia = np.trace(second)*np.eye(3)-second-mass*(np.dot(com, com)*np.eye(3)-np.outer(com, com))
        xyz = np.zeros(3) if origin is None else np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        rpy = np.zeros(3) if origin is None else np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
        if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite(np.r_[xyz, rpy]).all():
            raise ValueError('INVALID_PAN_MESH_ORIGIN')
        rotation = Rotation.from_euler('xyz', rpy).as_matrix()
        placed = vertices @ rotation.T+xyz
        if np.any(placed < low-1e-7) or np.any(placed > high+1e-7):
            raise ValueError('PAN_MESH_OUTSIDE_COLLISION_ENVELOPE')
        if not np.isfinite(inertia).all() or np.linalg.eigvalsh(inertia).min() <= 0:
            raise ValueError('INVALID_PAN_MESH_INERTIA')
        parts.append(dict(name=name, kind='mesh', vertices=vertices-com, triangles=faces,
            rotation=rotation, center=xyz+rotation@com, mass=mass,
            inertia=rotation @ inertia @ rotation.T))
    if not parts:
        raise ValueError('PAN_MESH_COMPONENTS_REQUIRED')
    return parts


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
    left = np.asarray(recipe.get('shaft_reference_in_tool_m',recipe['left_grip_in_tool_m']), dtype=float)
    if (not np.isfinite(np.r_[density, wall, size, center, radius, ends, axis, left]).all() or
            density <= 0 or radius <= 0 or wall <= 0 or left.shape != (3,) or np.any(size <= 2*wall) or
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
    if recipe.get('d_handle_urdf'):
        from bindu_kinematics.tool_geometry import d_handle_parts
        for part in d_handle_parts(recipe):
            r,l=part['radius'],part['length']
            m=density*np.pi*r*r*l
            local=m*np.diag([(3*r*r+l*l)/12]*2+[r*r/2])
            parts.append(dict(part,mass=m,inertia=part['rotation'] @ local @ part['rotation'].T))
    use_mesh = recipe.get('pan_from_urdf', False)
    if type(use_mesh) is not bool:
        raise ValueError('INVALID_PAN_GEOMETRY_SOURCE')
    if use_mesh:
        return parts+_urdf_pan_parts(recipe, density)
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

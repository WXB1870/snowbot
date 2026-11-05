"""Installed Snowbot PhysX scene construction; Isaac imports stay lazy."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from bindu_kinematics.tool_geometry import bin_boxes
from .physics import wooden_parts, compound_mass, joint_drive_parameters

def pose(position, wxyz):
    result = np.eye(4)
    result[:3, 3] = position
    result[:3, :3] = Rotation.from_quat(np.r_[wxyz[1:], wxyz[0]]).as_matrix()
    return result


def quaternion(matrix):
    xyzw = Rotation.from_matrix(matrix).as_quat()
    return np.r_[xyzw[3], xyzw[:3]]


def validate_grasp_config(cfg):
    model = cfg.get('grasp_model', 'proxy')
    if model not in ('proxy', 'contact'):
        raise ValueError('UNKNOWN_GRASP_MODEL')
    if model == 'contact':
        settings = cfg['contact_grasp']
        fields = ('joint_stiffness_nm_rad','joint_damping_nm_s_rad','max_effort_nm','axial_effort_nm','contact_offset_m')
        if (any(not np.isfinite(settings[k]) or settings[k] <= 0 for k in fields) or
                settings['axial_effort_nm'] > settings['max_effort_nm'] or
                settings['contact_offset_m'] > .001):
            raise ValueError('INVALID_CONTACT_GRASP_DRIVE')
        for key in ('closed_positions_rad','open_positions_rad'):
            if (set(settings[key]) != {'arm_l_joint_8','arm_r_joint_8'} or
                    not np.isfinite(list(settings[key].values())).all()):
                raise ValueError('INVALID_CONTACT_GRASP_JOINTS')


def build_scene(motion, cfg, output):
    from pxr import Gf, Vt, Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema, PhysicsSchemaTools, UsdLux
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation, RigidPrim
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.viewports import set_camera_view
    enable_extension('isaacsim.asset.importer.urdf')
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    source = Path(motion['display_urdf'])
    tree = ET.parse(source)
    prefix = 'package://snowbot_description/'
    for mesh in tree.findall('.//mesh'):
        uri = mesh.get('filename')
        if not uri.startswith(prefix):
            raise ValueError('UNEXPECTED_MESH_URI:'+uri)
        mesh.set('filename', str(source.parent.parent/uri.removeprefix(prefix)))
    absolute = output/'model.urdf'
    tree.write(absolute, encoding='utf-8', xml_declaration=True)
    usd = URDFImporter(URDFImporterConfig(urdf_path=str(absolute), usd_path=str(output/'usd'),
        fix_base=True, merge_fixed_joints=False, allow_self_collision=False,
        joint_drive_type='force', joint_target_type='position',
        override_joint_stiffness=cfg['joint_stiffness_nm_rad'],
        override_joint_damping=cfg['joint_damping_nm_s_rad'])).import_urdf()
    dt = cfg['physics_dt_s']
    world = World(stage_units_in_meters=1., physics_dt=dt, rendering_dt=dt*round(1/(cfg['gui_render_hz']*dt)))
    stage = world.stage
    model = stage.DefinePrim('/World/Robot', 'Xform')
    model.GetReferences().AddReference(str(usd))
    variant = model.GetVariantSets().GetVariantSet('Physics')
    if not variant.IsValid() or not variant.SetVariantSelection('physx'):
        raise ValueError('PHYSX_ASSET_VARIANT_REQUIRED')
    roots = [p for p in Usd.PrimRange(model) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise ValueError('ARTICULATION_ROOT_LAYOUT:'+str([str(p.GetPath()) for p in roots]))
    art = PhysxSchema.PhysxArticulationAPI.Apply(roots[0])
    art.CreateSolverPositionIterationCountAttr(cfg['solver_position_iterations'])
    art.CreateSolverVelocityIterationCountAttr(cfg['solver_velocity_iterations'])
    body = world.scene.add(SingleArticulation(str(roots[0].GetPath()), name='snowbot'))
    # PhysX uses float32 joint constraints. Keep numerical penetration of its
    # hard stops inside the public URDF bounds; never clip measured feedback.
    inset = float(cfg.get('joint_limit_inset_rad', 0.))
    if not np.isfinite(inset) or not 0 <= inset <= .001:
        raise ValueError('PHYSICS_JOINT_LIMIT_INSET')
    physical_limits = {}
    for prim in Usd.PrimRange(model):
        if not prim.IsA(UsdPhysics.RevoluteJoint):continue
        joint = UsdPhysics.RevoluteJoint(prim)
        lo, hi = joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()
        if lo is None or hi is None:continue
        lo, hi = np.deg2rad([lo, hi])
        if hi-lo <= 2*inset:raise ValueError('PHYSICS_JOINT_LIMIT_WIDTH')
        physical_limits[prim.GetName()] = [float(lo+inset),float(hi-inset)]
        if inset:
            joint.GetLowerLimitAttr().Set(float(np.rad2deg(lo+inset)))
            joint.GetUpperLimitAttr().Set(float(np.rad2deg(hi-inset)))
    # Robot self-collision is checked independently against CAD. Tool/robot
    # contact stays enabled in contact mode; legacy proxy mode is explicit.
    links = {p.GetName(): p for p in Usd.PrimRange(model) if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    contact_grasp = cfg.get('grasp_model', 'proxy') == 'contact'
    grip_colliders = []
    if contact_grasp:
        # Edit only generated USD. Keep the original CAD and URDF unchanged.
        jaw_names = {f'arm_{s}_link_{n}' for s in 'lr' for n in (7, 8)}
        for prim in list(Usd.PrimRange(model)):
            if prim.IsInstance() and any(n in str(prim.GetPath()).split('/') for n in jaw_names):
                prim.SetInstanceable(False)
        for prim in Usd.PrimRange(model):
            if (prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsA(UsdGeom.Mesh) and
                    any(n in str(prim.GetPath()).split('/') for n in jaw_names)):
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexDecomposition')
                decomp = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
                decomp.CreateMaxConvexHullsAttr(64)
                decomp.CreateHullVertexLimitAttr(64)
                decomp.CreateVoxelResolutionAttr(500000)
                decomp.CreateErrorPercentageAttr(.1)
                decomp.CreateShrinkWrapAttr(True)
                contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                contact.CreateContactOffsetAttr(cfg['contact_grasp']['contact_offset_m'])
                contact.CreateRestOffsetAttr(0.)
                grip_colliders.append(str(prim.GetPath()))
        if len(grip_colliders) != 4:
            raise ValueError('GRIP_COLLIDER_LAYOUT:'+str(grip_colliders))
    # Massless URDF frames remain Xforms in 6.2. Apply each grasp to its
    # physical ancestor and retain the exact fixed-frame transform.
    joints_by_child = {j.find('child').get('link'):j for j in tree.findall('joint')}
    flanges, flange_mounts = [], []
    for side in ('left', 'right'):
        name = motion[side+'_frame']
        mount = np.eye(4)
        while name not in links:
            joint = joints_by_child[name]
            if joint.get('type') != 'fixed':
                raise ValueError('NONRIGID_FLANGE_ANCESTOR:'+name)
            origin = joint.find('origin')
            fixed = np.eye(4)
            if origin is not None:
                fixed[:3,3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                fixed[:3,:3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            mount = fixed @ mount
            name = joint.find('parent').get('link')
        flanges.append(links[name])
        flange_mounts.append(mount)
    tool = UsdGeom.Xform.Define(stage, '/World/Shovel')
    tool_prim = tool.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(tool_prim)
    rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(tool_prim)
    rigid_api.CreateLinearDampingAttr(0.)
    rigid_api.CreateAngularDampingAttr(0.)
    parts = wooden_parts(motion['recipe'], cfg['material'])
    mass, com, inertia = compound_mass(parts)
    eigenvalues, principal = np.linalg.eigh(inertia)
    if np.linalg.det(principal) < 0:
        principal[:, 0] *= -1
    api = UsdPhysics.MassAPI.Apply(tool_prim)
    api.CreateMassAttr(mass)
    api.CreateCenterOfMassAttr(Gf.Vec3f(*com))
    api.CreateDiagonalInertiaAttr(Gf.Vec3f(*eigenvalues))
    api.CreatePrincipalAxesAttr(Gf.Quatf(quaternion(principal)[0], Gf.Vec3f(*quaternion(principal)[1:])))
    material = UsdShade.Material.Define(stage, '/World/WoodMaterial')
    mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    for key, creator in (('static_friction', mat.CreateStaticFrictionAttr),
                         ('dynamic_friction', mat.CreateDynamicFrictionAttr),
                         ('restitution', mat.CreateRestitutionAttr)):
        creator(cfg['material'][key])
    for part in parts:
        path = '/World/Shovel/'+part['name']
        if part['kind'] == 'box':
            shape = UsdGeom.Cube.Define(stage, path)
            shape.CreateSizeAttr(1.)
            scale = part['size']
        elif part['kind'] == 'mesh':
            shape = UsdGeom.Mesh.Define(stage, path)
            shape.CreatePointsAttr(Vt.Vec3fArray(part['vertices'].tolist()))
            shape.CreateFaceVertexCountsAttr([3]*len(part['triangles']))
            shape.CreateFaceVertexIndicesAttr(part['triangles'].ravel().tolist())
            shape.CreateSubdivisionSchemeAttr('none')
            scale = np.ones(3)
        else:
            shape = UsdGeom.Cylinder.Define(stage, path)
            shape.CreateRadiusAttr(part['radius'])
            shape.CreateHeightAttr(part['length'])
            shape.CreateAxisAttr('Z')
            scale = np.ones(3)
        tf = np.eye(4)
        tf[:3, :3] = part['rotation'] @ np.diag(scale)
        tf[:3, 3] = part['center']
        shape.AddTransformOp().Set(Gf.Matrix4d(tf.T.tolist()))
        shape.CreateDisplayColorAttr([Gf.Vec3f(.55, .33, .15)])
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        if part['kind'] == 'mesh':
            UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr('convexDecomposition')
            decomposition = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(shape.GetPrim())
            decomposition.CreateMaxConvexHullsAttr(64)
            decomposition.CreateHullVertexLimitAttr(64)
            decomposition.CreateVoxelResolutionAttr(500000)
            decomposition.CreateErrorPercentageAttr(.1)
            decomposition.CreateShrinkWrapAttr(True)
        contact = PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
        contact.CreateContactOffsetAttr(.001)
        contact.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material, materialPurpose='physics')
    for path in grip_colliders:
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(material, materialPurpose='physics')
    if not contact_grasp:
        UsdPhysics.FilteredPairsAPI.Apply(tool_prim).CreateFilteredPairsRel().SetTargets(
            [link.GetPath() for link in links.values()])
    z = motion['recipe']['ground']['z_in_base_m']-cfg.get('diagnostic_ground_drop_m',0.)
    PhysicsSchemaTools.addGroundPlane(stage, '/World/Ground', 'Z', 5., Gf.Vec3f(0., 0., z), Gf.Vec3f(.5))
    for name, size, center in bin_boxes(motion['recipe']):
        cube = UsdGeom.Cube.Define(stage, '/World/Bin/'+name)
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(.25, .4, .3)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    view = world.scene.add(RigidPrim([str(p.GetPath()) for p in flanges]+['/World/Shovel'], name='grips'))
    contact_paths = [str(p.GetPath()) for p in stage.Traverse()
                     if p.HasAPI(UsdPhysics.CollisionAPI) and
                     (str(p.GetPath()).startswith('/World/Ground/') or str(p.GetPath()).startswith('/World/Bin/'))]
    contacts = {'environment': world.scene.add(RigidPrim('/World/Shovel', name='tool_contacts',
        contact_filter_prim_paths_expr=contact_paths, max_contact_count=1024, disable_stablization=False))}
    if contact_grasp:
        for side, short in (('left', 'l'), ('right', 'r')):
            contacts[side] = world.scene.add(RigidPrim('/World/Shovel', name=side+'_contacts',
                contact_filter_prim_paths_expr=[str(links[f'arm_{short}_link_{n}'].GetPath()) for n in (7,8)],
                max_contact_count=2048, disable_stablization=False))
    UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1000.)
    set_camera_view(eye=cfg.get('camera_eye_m', [3., 2.4, 2.1]),
                    target=cfg.get('camera_target_m', [.6, -.3, .8]))
    world.get_physics_context().set_gravity(-9.81)
    from isaacsim.core.simulation_manager import SimulationManager
    SimulationManager.enable_fabric(cfg.get('use_fabric', True))
    world.reset()
    if SimulationManager.get_active_physics_engine() != 'physx':
        raise ValueError('PHYSX_BACKEND_REQUIRED')
    names = body.dof_names
    limits = {j.get('name'):j.find('limit') for j in tree.findall('joint') if j.get('type') != 'fixed'}
    if set(names) != set(limits):
        raise ValueError('PHYSICS_JOINT_LAYOUT')
    controller = body.get_articulation_controller()
    kp, kd, drive_efforts = joint_drive_parameters(names, limits, cfg)
    controller.set_gains(kp, kd)
    controller.set_max_efforts(drive_efforts)
    gains = controller.get_gains()
    if not np.allclose(gains[0], kp) or not np.allclose(gains[1], kd):
        raise ValueError('DRIVE_GAIN_READBACK')
    if contact_grasp:
        settings = cfg['contact_grasp']
        kp, kd = np.array(gains[0]), np.array(gains[1])
        efforts = drive_efforts.copy()
        for n in settings['closed_positions_rad']:
            i = names.index(n)
            for key in ('closed_positions_rad','open_positions_rad'):
                if not float(limits[n].get('lower')) <= settings[key][n] <= float(limits[n].get('upper')):
                    raise ValueError('GRIP_JOINT_LIMIT:'+n)
            kp[i], kd[i] = settings['joint_stiffness_nm_rad'], settings['joint_damping_nm_s_rad']
            efforts[i] = min(efforts[i], settings['max_effort_nm'])
        controller.set_gains(kp, kd)
        controller.set_max_efforts(efforts)
        gains = controller.get_gains()
    if not np.isclose(view.get_masses()[2], mass, rtol=1e-5):
        raise ValueError('TOOL_MASS_READBACK')
    if list(view.prim_paths) != [str(p.GetPath()) for p in flanges]+['/World/Shovel']:
        raise ValueError('RIGID_VIEW_ORDER')
    source = dict(joint_limit_inset_rad=inset,physical_joint_limits_rad=physical_limits,
                  mass_kg=mass, center_of_mass_in_tool_m=com.tolist(), inertia_at_com_kg_m2=inertia.tolist(),
                  parts=[dict(name=p['name'], mass_kg=p['mass']) for p in parts],
                  dof_names=names, drive_effort_limits_nm=drive_efforts.tolist(),
                  gains_readback=[g.tolist() for g in gains],
                  articulation_root=str(roots[0].GetPath()), flanges=[str(p.GetPath()) for p in flanges],
                  flange_from_physical_body=[m.tolist() for m in flange_mounts],
                  environment_contact_paths=contact_paths)
    source['native_mass_kg'] = float(view.get_masses()[2])
    source['fabric_enabled'] = SimulationManager.is_fabric_enabled()
    source['native_com_in_tool_m'] = np.asarray(view.get_coms()[0]).reshape(-1,3)[2].tolist()
    source['grip_colliders'] = grip_colliders
    source['tool_robot_contacts_enabled'] = contact_grasp
    if not np.allclose(source['native_com_in_tool_m'], com, atol=1e-6):
        raise ValueError('TOOL_COM_READBACK')
    return world, body, view, contacts, source

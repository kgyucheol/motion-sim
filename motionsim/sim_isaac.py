"""Isaac Sim backend for the Decoupled WBC controller.

Import only after `isaacsim.SimulationApp` has started (see motionsim/isaac_worker.py).

Robot model (integrations/robot-models.json `isaac_actuators`): mass, inertia and geometry come from
the model URDF; motor properties follow the policy model g1_gear_wbc.xml:
  - explicit torque control: PhysX drives have zero stiffness/damping; torques come from the controller
  - joint armature 0.01 (native PhysX joint armature)
  - effort limits from the policy model (ankle and waist roll/pitch 50 Nm instead of the URDF's 35 Nm)
  - no joint velocity limit (MuJoCo has none; the URDF's 37 rad/s limit is removed)
These joint properties are authored on the USD joints before the simulation starts. Setting armature
through the tensor API at runtime reads back correctly but does not change the dynamics (measured
2026-09-28: wrist roll joints went unstable at +-37 rad/s within 3 steps).
  - MuJoCo's passive joint damping (0.001) and dry friction (frictionloss 0.1 Nm) are added to the
    commanded torque, with a smooth tanh approximation of dry friction. PhysX joint friction is unitless
    and is left at zero.
Contact: foot colliders are the URDF's four 5 mm spheres per foot, identical to the policy model. The
ground uses friction 1.0 with `max` combine mode so the effective coefficient matches MuJoCo's default 1.0.
"""
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from .robot import FEET, SOLE_CORNERS, Robot
from .wbc import ARMS, EFFORT_LIMITS, RobotState, load_parameters

ARMATURE = .01
MAX_JOINT_VELOCITY = 1e3      # rad/s, effectively unlimited
PASSIVE_DAMPING = .001
DRY_FRICTION = .1
DRY_FRICTION_VELOCITY = .02   # rad/s scale of the tanh approximation


def author_joint_properties(stage, effort_limits):
    """Author motor properties on the robot's revolute joints. Returns the joint count."""
    from pxr import PhysxSchema, UsdPhysics
    joints = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.RevoluteJoint) or prim.GetName() not in effort_limits:
            continue
        physx = PhysxSchema.PhysxJointAPI.Apply(prim)
        physx.CreateArmatureAttr().Set(ARMATURE)
        physx.CreateMaxJointVelocityAttr().Set(MAX_JOINT_VELOCITY)
        physx.CreateJointFrictionAttr().Set(0.)
        drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
        drive.CreateStiffnessAttr().Set(0.)
        drive.CreateDampingAttr().Set(0.)
        drive.CreateMaxForceAttr().Set(float(effort_limits[prim.GetName()]))
        joints += 1
    return joints


_ASSET_ROOT = None
_ASSET_UNITS = {}


def asset_root():
    """Isaac asset root: the configured Nucleus/S3 root when available, else the catalog default."""
    global _ASSET_ROOT
    if _ASSET_ROOT is None:
        from .scene import CATALOG
        try:
            from isaacsim.storage.native import get_assets_root_path
            configured = get_assets_root_path()
        except Exception:
            configured = None
        _ASSET_ROOT = (configured.rstrip('/') + '/') if configured else CATALOG['asset_root']
    return _ASSET_ROOT


def asset_scale_correction(prim, asset):
    """Scale that makes the referenced asset match its catalog size.

    Asset geometry values are treated as metres: packing_table.usd declares metersPerUnit 0.01 but its
    coordinates are already metre-sized (a 2.47 m table). Measuring the referenced prim in this stage
    makes the result independent of whether Kit rescales references with different units.
    """
    from pxr import Usd, UsdGeom
    from .scene import CATALOG_PATHS
    if asset not in _ASSET_UNITS:
        measured = np.asarray(UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render'])
                              .ComputeLocalBound(prim).ComputeAlignedRange().GetSize(), dtype=float)
        expected = np.asarray(CATALOG_PATHS[asset]['size_m'], dtype=float)
        _ASSET_UNITS[asset] = float(np.median(expected / np.maximum(measured, 1e-9)))
    return _ASSET_UNITS[asset]


def export_asset_mesh(asset):
    """Triangle mesh of an Isaac asset in its default prim's frame (the frame the scene pose sets); values are metres."""
    from pxr import Gf, Usd, UsdGeom
    stage = Usd.Stage.Open(asset_root() + asset)
    if stage is None:
        raise FileNotFoundError(f'Isaac 에셋을 열 수 없습니다: {asset}')
    root = stage.GetDefaultPrim() or stage.GetPseudoRoot()
    cache = UsdGeom.XformCache()
    to_root = cache.GetLocalToWorldTransform(root).GetInverse() if root != stage.GetPseudoRoot() else Gf.Matrix4d(1.)
    vertices, faces, offset = [], [], 0
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh) or UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        mesh = UsdGeom.Mesh(prim)
        points, counts, indices = mesh.GetPointsAttr().Get(), mesh.GetFaceVertexCountsAttr().Get(), mesh.GetFaceVertexIndicesAttr().Get()
        if not points or not counts:
            continue
        matrix = np.array(cache.GetLocalToWorldTransform(prim) * to_root)
        homogeneous = np.c_[np.asarray(points, dtype=float), np.ones(len(points))]
        vertices.append((homogeneous @ matrix)[:, :3])
        indices, counts = np.asarray(indices, dtype=np.int64), np.asarray(counts)
        if np.all(counts == 3):
            faces.append(indices.reshape(-1, 3) + offset)
        else:                    # fan-triangulate polygons
            starts = np.r_[0, np.cumsum(counts)[:-1]]
            faces.append(np.array([(indices[s], indices[s + k], indices[s + k + 1])
                                   for s, count in zip(starts, counts) for k in range(1, count - 1)]).reshape(-1, 3) + offset)
        offset += len(points)
    if not vertices:
        raise ValueError(f'Isaac 에셋에 메시가 없습니다: {asset}')
    return np.vstack(vertices), np.vstack(faces).astype(np.int32)


def _rigid_part(matrix):
    """Drop per-axis scale from a 4x4 world transform (column-vector convention), keeping rotation and translation.

    Asset prims can carry extra non-uniform scale ops (e.g. `xformOp:scale:unitsResolve`), so each axis is
    normalised separately rather than assuming a uniform scale.
    """
    out = np.eye(4)
    axes = matrix[:3, :3]
    out[:3, :3] = Rotation.from_matrix(axes / np.linalg.norm(axes, axis=0)).as_matrix()
    out[:3, 3] = matrix[:3, 3]
    return out


def import_robot_urdf(urdf_path):
    import omni.kit.commands
    from isaacsim.asset.importer.urdf import _urdf
    config = _urdf.ImportConfig()
    config.set_fix_base(False)
    config.set_merge_fixed_joints(False)
    config.set_make_default_prim(False)
    config.set_self_collision(False)
    config.set_import_inertia_tensor(True)
    config.set_create_physics_scene(False)
    config.set_default_drive_strength(0.)
    config.set_default_position_drive_damping(0.)
    ok, prim_path = omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=str(urdf_path),
                                              import_config=config, get_articulation_root=True)
    if not ok:
        raise RuntimeError(f'URDF import failed: {urdf_path}')
    return prim_path


class IsaacBackend:
    def __init__(self, robot: Robot, parameters=None):
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from pxr import PhysxSchema, UsdPhysics
        self.robot, self.p = robot, parameters or load_parameters()
        self.world = World(physics_dt=self.p['simulation_dt'], rendering_dt=self.p['simulation_dt'] * 4,
                           stage_units_in_meters=1.)
        ground = self.world.scene.add_default_ground_plane(static_friction=1., dynamic_friction=1., restitution=0.)
        material = ground.get_applied_physics_material().prim
        PhysxSchema.PhysxMaterialAPI.Apply(material).CreateFrictionCombineModeAttr().Set('max')
        UsdPhysics.MaterialAPI(material)  # keep the explicit friction values authored above
        self.prim_path = import_robot_urdf(robot.urdf_path)
        import omni.usd
        authored = author_joint_properties(omni.usd.get_context().get_stage(), dict(zip(robot.names, EFFORT_LIMITS)))
        if authored != 29:
            raise ValueError(f'Expected 29 revolute joints in the imported robot, found {authored}')
        self.articulation = self.world.scene.add(SingleArticulation(self.prim_path, name='robot'))
        self.world.reset()
        names = list(self.articulation.dof_names)
        if sorted(names) != sorted(robot.names):
            raise ValueError('Isaac articulation joints differ from the URDF revolute joints')
        # Isaac orders DOFs by articulation traversal; motion-sim uses URDF declaration order.
        self.to_isaac = np.array([names.index(name) for name in robot.names])
        self.pin_data = robot.model.createData()
        self.objects, self.object_handles, self.object_offsets = [], {}, {}

    def set_scene(self, objects):
        """Replace the scene objects (motion-sim scene contract). Takes effect at the next reset()."""
        import omni.usd
        from isaacsim.core.api.materials import PhysicsMaterial
        from isaacsim.core.api.objects import (DynamicCuboid, DynamicCylinder, DynamicSphere, FixedCuboid, FixedCylinder,
                                               FixedSphere)
        from isaacsim.core.prims import SingleRigidPrim, SingleXFormPrim
        from isaacsim.core.utils.stage import add_reference_to_stage
        from pxr import Usd, UsdPhysics
        stage = omni.usd.get_context().get_stage()
        self.world.stop()   # never edit rigid bodies under a running PhysX scene; reset() restarts it
        for name in list(self.object_handles):
            if self.world.scene.object_exists(name):
                self.world.scene.remove_object(name)
        for path in ('/World/objects', '/World/object_materials'):
            if stage.GetPrimAtPath(path):
                stage.RemovePrim(path)
        self.objects, self.object_handles, self.object_offsets = [dict(item) for item in objects], {}, {}
        classes = {('box', False): DynamicCuboid, ('box', True): FixedCuboid, ('sphere', False): DynamicSphere,
                   ('sphere', True): FixedSphere, ('cylinder', False): DynamicCylinder, ('cylinder', True): FixedCylinder}
        for item in self.objects:
            path, name = f'/World/objects/{item["id"]}', item['id']
            position, wxyz, fixed = np.asarray(item['position'], dtype=float), np.asarray(item['wxyz'], dtype=float), item['fixed']
            if item['kind'] != 'asset':
                size = np.asarray(item['size'], dtype=float)
                material = PhysicsMaterial(f'/World/object_materials/{name}', static_friction=item['friction'],
                                           dynamic_friction=item['friction'], restitution=0.)
                color = np.array([int(item['color'][i:i + 2], 16) for i in (1, 3, 5)]) / 255
                arguments = {'prim_path': path, 'name': name, 'position': position, 'orientation': wxyz, 'color': color,
                             'physics_material': material}
                if item['kind'] == 'box':
                    arguments.update(size=1., scale=size)
                elif item['kind'] == 'sphere':
                    arguments.update(radius=size[0] / 2)
                else:
                    arguments.update(radius=size[0] / 2, height=size[2])
                if not fixed:
                    arguments['mass'] = item['mass_kg']
                self.object_handles[name] = self.world.scene.add(classes[item['kind'], fixed](**arguments))
                continue
            add_reference_to_stage(usd_path=asset_root() + item['asset'], prim_path=path)
            root = SingleXFormPrim(path, name=f'{name}_xform')
            root.set_local_scale(np.ones(3))
            correction = asset_scale_correction(stage.GetPrimAtPath(path), item['asset'])
            root.set_world_pose(position=position, orientation=wxyz)
            root.set_local_scale(np.full(3, float(item['scale']) * correction))
            rigid = [p for p in Usd.PrimRange(stage.GetPrimAtPath(path), Usd.TraverseInstanceProxies())
                     if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            if fixed:
                for prim in rigid:
                    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
                self.object_handles[name] = root
                continue
            if not rigid:
                UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(path))
                rigid = [stage.GetPrimAtPath(path)]
            if item.get('mass_kg') is not None:
                UsdPhysics.MassAPI.Apply(rigid[0]).CreateMassAttr().Set(float(item['mass_kg']))
            # Keep the body's authored transform: the default reset_xform_properties=True rewrites it and moved
            # packing_table.usd's rigid child by 13 cm (measured 2026-09-28).
            self.object_handles[name] = self.world.scene.add(
                SingleRigidPrim(str(rigid[0].GetPath()), name=name, reset_xform_properties=False))
            if rigid[0].GetPath() != stage.GetPrimAtPath(path).GetPath():
                # The rigid body can sit on a child prim; report poses of the asset root the editor placed.
                from pxr import UsdGeom
                cache = UsdGeom.XformCache()
                # PhysX reports unscaled rigid poses, so strip the (uniform) scale before relating the frames.
                body = _rigid_part(np.array(cache.GetLocalToWorldTransform(rigid[0])).T)
                root_matrix = _rigid_part(np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T)
                self.object_offsets[name] = np.linalg.inv(body) @ root_matrix

    def object_poses(self):
        """{object id: (position, wxyz)} of each object's root frame (the frame the editor placed)."""
        poses = {}
        for item in self.objects:
            position, wxyz = self.object_handles[item['id']].get_world_pose()
            position, wxyz = np.asarray(position, dtype=float), np.asarray(wxyz, dtype=float)
            if item['id'] in self.object_offsets:
                body = np.eye(4)
                body[:3, :3] = Rotation.from_quat(wxyz[[1, 2, 3, 0]]).as_matrix()
                body[:3, 3] = position
                root = body @ self.object_offsets[item['id']]
                position, wxyz = root[:3, 3], Rotation.from_matrix(root[:3, :3]).as_quat()[[3, 0, 1, 2]]
            poses[item['id']] = (position.tolist(), wxyz.tolist())
        return poses

    def _isaac_order(self, values):
        out = np.zeros(29)
        out[self.to_isaac] = values
        return out

    def settings(self):
        return {'armature': ARMATURE, 'max_joint_velocity': MAX_JOINT_VELOCITY,
                'effort_limits_nm': EFFORT_LIMITS.tolist(),
                'passive': {'damping': PASSIVE_DAMPING, 'dry_friction_nm': DRY_FRICTION, 'tanh_scale': DRY_FRICTION_VELOCITY},
                'solver_position_iterations': int(self.articulation.get_solver_position_iteration_count()),
                'solver_velocity_iterations': int(self.articulation.get_solver_velocity_iteration_count()),
                'physics_dt': self.p['simulation_dt'], 'ground_friction': 1.0,
                'asset_scale_corrections': dict(_ASSET_UNITS)}

    def reset(self, joint_positions, root_xy_yaw=(0., 0., 0.)):
        """Place the robot with the given joints and its lowest sole corner on the ground."""
        q = self.robot.home.copy()
        q[:3] = [root_xy_yaw[0], root_xy_yaw[1], 0.]
        q[3:7] = Rotation.from_euler('z', root_xy_yaw[2]).as_quat()[[3, 0, 1, 2]]
        q[7:] = joint_positions
        d = self.robot.data(q)
        q[2] = -min(self.robot.point(d, k)[0][2] + (self.robot.point(d, k)[1] @ np.array(o))[2]
                    for k in FEET for o in SOLE_CORNERS) + .001
        self.world.reset()
        self.articulation.set_world_pose(position=q[:3], orientation=q[3:7])
        self.articulation.set_joint_positions(self._isaac_order(q[7:]))
        self.articulation.set_joint_velocities(np.zeros(29))
        self.articulation.set_linear_velocity(np.zeros(3))
        self.articulation.set_angular_velocity(np.zeros(3))
        self._torques = np.zeros(29)

    def _root(self):
        position, wxyz = self.articulation.get_world_pose()
        rotation = Rotation.from_quat(np.asarray(wxyz)[[1, 2, 3, 0]])
        linear = rotation.inv().apply(self.articulation.get_linear_velocity())
        angular = rotation.inv().apply(self.articulation.get_angular_velocity())
        return np.asarray(position, dtype=float), np.asarray(wxyz, dtype=float), linear, angular

    def state(self):
        position, wxyz, _, angular = self._root()
        q = np.asarray(self.articulation.get_joint_positions(), dtype=float)[self.to_isaac]
        dq = np.asarray(self.articulation.get_joint_velocities(), dtype=float)[self.to_isaac]
        return RobotState(np.r_[position, wxyz, q], angular, dq)

    def arm_bias(self):
        """Gravity + Coriolis torque on the arms from the same URDF model (Pinocchio RNEA, zero acceleration)."""
        position, wxyz, linear, angular = self._root()
        state = self.state()
        v = np.zeros(self.robot.model.nv)
        v[:3], v[3:6] = linear, angular          # Pinocchio free-flyer velocity is expressed in the root frame
        v[self.robot.pin_v_index] = state.joint_velocity
        tau = pin.rnea(self.robot.model, self.pin_data, self.robot.to_pin(state.qpos), v, np.zeros(self.robot.model.nv))
        return tau[self.robot.pin_v_index][ARMS]

    def step(self, torques):
        dq = np.asarray(self.articulation.get_joint_velocities(), dtype=float)[self.to_isaac]
        passive = -PASSIVE_DAMPING * dq - DRY_FRICTION * np.tanh(dq / DRY_FRICTION_VELOCITY)
        self.articulation.set_joint_efforts(self._isaac_order(np.asarray(torques) + passive))
        self.world.step(render=False)


def rollout(controller, backend, seconds, progress=None):
    """Run the controller from its initial pose. Returns (recording, fell)."""
    backend.reset(controller.initial_joint_positions())
    controller.reset(backend.state())
    steps = int(round(seconds / controller.p['simulation_dt']))
    for index in range(steps):
        state = backend.state()
        backend.step(controller.compute_torques(state, backend.arm_bias()))
        recorded = len(controller.recording)
        alive = controller.after_step(backend.state())
        if backend.objects and len(controller.recording) > recorded:
            controller.recording[-1]['objects'] = backend.object_poses()
        if not alive:
            return controller.recording, True
        if progress is not None and index % 100 == 0:
            progress(index / steps)
    return controller.recording, False

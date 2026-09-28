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
                'physics_dt': self.p['simulation_dt'], 'ground_friction': 1.0}

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
        if not controller.after_step(backend.state()):
            return controller.recording, True
        if progress is not None and index % 100 == 0:
            progress(index / steps)
    return controller.recording, False

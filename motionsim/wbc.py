"""GR00T Decoupled WBC controller, independent of the physics engine.

Ported from Motion Creator `motioncreator/decoupled_wbc.py` (auto mode, no grasp feedback yet):
  - lower body (legs + waist, 15 DoF): Balance/Walk ONNX policy at 50 Hz -> PD torque at 200 Hz
  - arms (14 DoF): PD tracking of the authored motion plus the simulator's bias torque
  - authored root trajectory -> local velocity / yaw-rate / height commands (never writes root state)
  - authored waist joints -> torso RPY command

A physics backend owns the robot. Every physics step it calls
    torques = controller.compute_torques(state, arm_bias)
    <step physics>
    controller.after_step(state_after)
`state` uses the motion-sim conventions: qpos [xyz, wxyz, 29 joints in URDF order], root angular
velocity in the root frame (MuJoCo free-joint convention), joint velocities in the same order.
"""
from collections import deque
from dataclasses import dataclass, field
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
PARAMETERS_PATH = ROOT / 'integrations/decoupled-wbc-parameters.json'
ASSET_MANIFEST = ROOT / 'integrations/decoupled-wbc-assets.json'
LOWER = slice(0, 15)     # joint indices: 6 left leg, 6 right leg, 3 waist
ARMS = slice(15, 29)     # 7 left arm, 7 right arm
WAIST = slice(12, 15)    # waist yaw, roll, pitch
RECORD_EVERY = 8         # 200 Hz / 8 = 25 Hz, like Motion Creator
# Policy-model actuator limits (g1_gear_wbc.xml actuatorfrcrange), URDF order.
EFFORT_LIMITS = np.array([88, 139, 88, 139, 50, 50, 88, 139, 88, 139, 50, 50, 88, 50, 50,
                          25, 25, 25, 25, 25, 5, 5, 25, 25, 25, 25, 25, 5, 5], dtype=float)


def load_parameters():
    return json.loads(PARAMETERS_PATH.read_text())


def asset_path(name):
    manifest = json.loads(ASSET_MANIFEST.read_text())
    return ROOT / manifest['target_dir'] / name


def verify_assets():
    manifest = json.loads(ASSET_MANIFEST.read_text())
    for asset in manifest['assets']:
        path = ROOT / manifest['target_dir'] / asset['file']
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != asset['sha256']:
            raise FileNotFoundError(f'Decoupled WBC 자산이 없거나 해시가 다릅니다: {path}. scripts/setup-decoupled-wbc.sh를 실행하세요.')


def quat_rotate_inverse(wxyz, vector):
    return Rotation.from_quat(np.asarray(wxyz)[[1, 2, 3, 0]]).inv().apply(vector)


def authored_waist_to_torso_rpy(waist_yaw_roll_pitch, limits):
    """Convert the editor's yaw/roll/pitch waist joints into policy XYZ torso RPY."""
    yaw, roll, pitch = np.asarray(waist_yaw_roll_pitch, dtype=float)
    rpy = Rotation.from_euler('zxy', [yaw, roll, pitch]).as_euler('xyz')
    return np.clip(rpy, -limits, limits).astype(np.float32)


def _yaw(wxyz):
    return float(Rotation.from_quat(np.asarray(wxyz)[[1, 2, 3, 0]]).as_euler('xyz')[2])


def _wrap_angle(value):
    return float((value + np.pi) % (2 * np.pi) - np.pi)


def ghost_tracking_command(current, reference, following, current_origin, reference_origin, dt, parameters):
    """Convert a ghost root trajectory into policy inputs without writing robot state."""
    ref0_yaw, world0_yaw = _yaw(reference_origin[3:7]), _yaw(current_origin[3:7])
    alignment = world0_yaw - ref0_yaw
    c, s = np.cos(alignment), np.sin(alignment)
    align_rotation = np.array([[c, -s], [s, c]])
    target_xy = current_origin[:2] + align_rotation @ (reference[:2] - reference_origin[:2])
    desired_world_velocity = align_rotation @ ((following[:2] - reference[:2]) / dt)
    current_yaw = _yaw(current[3:7])
    target_yaw = world0_yaw + _wrap_angle(_yaw(reference[3:7]) - ref0_yaw)
    controlled_world_velocity = desired_world_velocity + float(parameters['auto_position_gain']) * (target_xy - current[:2])
    cb, sb = np.cos(current_yaw), np.sin(current_yaw)
    local_velocity = np.array([[cb, sb], [-sb, cb]]) @ controlled_world_velocity
    yaw_rate = (_wrap_angle(_yaw(following[3:7]) - _yaw(reference[3:7])) / dt
                + float(parameters['auto_yaw_gain']) * _wrap_angle(target_yaw - current_yaw))
    limits = parameters['command_limits']
    nav = np.array([local_velocity[0], local_velocity[1], yaw_rate], dtype=np.float32)
    nav[:2] = np.clip(nav[:2], -limits['linear_velocity'], limits['linear_velocity'])
    nav[2] = np.clip(nav[2], -limits['angular_velocity'], limits['angular_velocity'])
    height = float(np.clip(parameters['initial_height'] + reference[2] - reference_origin[2],
                           limits['minimum_height'], limits['maximum_height']))
    tracking = {'root_error_m': float(np.linalg.norm(target_xy - current[:2])),
                'yaw_error_deg': abs(float(np.rad2deg(_wrap_angle(target_yaw - current_yaw))))}
    return nav, height, tracking


@dataclass
class CommandState:
    nav: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    height: float = .74
    torso_offset_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))


@dataclass
class RobotState:
    qpos: np.ndarray          # (36,) xyz, wxyz, joints (URDF order)
    root_angular_velocity: np.ndarray   # (3,) in the root frame
    joint_velocity: np.ndarray          # (29,)


def build_observation(state: RobotState, action, command: CommandState, torso_rpy, parameters):
    default = np.zeros(29, dtype=np.float32)
    default[LOWER] = parameters['default_angles']
    command_vector = np.zeros(7, dtype=np.float32)
    command_vector[:3] = command.nav * np.asarray(parameters['command_scale'], dtype=np.float32)
    command_vector[3] = command.height
    command_vector[4:7] = torso_rpy
    observation = np.zeros(86, dtype=np.float32)
    observation[:7] = command_vector
    observation[7:10] = state.root_angular_velocity * parameters['angular_velocity_scale']
    observation[10:13] = quat_rotate_inverse(state.qpos[3:7], np.array([0., 0., -1.]))
    observation[13:42] = (state.qpos[7:36] - default) * parameters['dof_position_scale']
    observation[42:71] = state.joint_velocity * parameters['dof_velocity_scale']
    observation[71:86] = action
    return observation


class LowerBodyPolicy:
    def __init__(self, parameters):
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        self.balance = ort.InferenceSession(str(asset_path('GR00T-WholeBodyControl-Balance.onnx')), options,
                                            providers=['CPUExecutionProvider'])
        self.walk = ort.InferenceSession(str(asset_path('GR00T-WholeBodyControl-Walk.onnx')), options,
                                         providers=['CPUExecutionProvider'])
        self.parameters = parameters
        self.history = deque(maxlen=parameters['observation_history'])
        self.reset()

    def reset(self):
        self.action = np.zeros(15, dtype=np.float32)
        self.history.clear()
        for _ in range(self.parameters['observation_history']):
            self.history.append(np.zeros(86, dtype=np.float32))

    def infer(self, observation, moving):
        self.history.append(observation)
        stacked = np.concatenate(tuple(self.history), dtype=np.float32)[None, :]
        session = self.walk if moving else self.balance
        name = session.get_inputs()[0].name
        self.action = np.asarray(session.run(None, {name: stacked})[0], dtype=np.float32).reshape(15)


class DecoupledController:
    """Auto-mode Decoupled WBC tracking one authored motion sampled at 50 Hz."""

    def __init__(self, reference_qpos, parameters=None, effort_limits=EFFORT_LIMITS):
        self.p = parameters or load_parameters()
        self.reference = np.asarray(reference_qpos, dtype=float)
        if self.reference.ndim != 2 or self.reference.shape[1] != 36:
            raise ValueError('reference_qpos must have shape [T, 36] sampled at 50 Hz')
        self.duration = (len(self.reference) - 1) / 50
        self.effort_limits = np.asarray(effort_limits, dtype=float)
        self.policy = LowerBodyPolicy(self.p)
        self.reset(None)

    def initial_joint_positions(self):
        """Legs and waist at the policy's default angles, arms at the authored start pose."""
        joints = np.zeros(29)
        joints[LOWER] = self.p['default_angles']
        joints[ARMS] = self.reference[0, 7:][ARMS]
        return joints

    def reset(self, state: RobotState | None):
        self.policy.reset()
        self.command = CommandState(height=self.p['initial_height'])
        self.step_count = 0
        self.play_time = 0.
        self.motion_time = 0.
        self.phase = 'settling'
        self.torso_rpy = np.zeros(3, dtype=np.float32)
        self.tracking = {'root_error_m': 0., 'yaw_error_deg': 0.}
        self.world_origin = state.qpos[:7].copy() if state is not None else np.r_[0, 0, 0, 1, 0, 0, 0.]
        self.recording = []

    def _reference_at(self, seconds):
        return self.reference[min(int(round(seconds * 50)), len(self.reference) - 1)]

    def _arm_target(self):
        if self.play_time < self.p['upper_body_settle_seconds']:
            return self._reference_at(0.)[7:][ARMS]
        return self._reference_at(self.motion_time)[7:][ARMS]

    def compute_torques(self, state: RobotState, arm_bias):
        """Joint torques (URDF order) for the next physics step."""
        p = self.p
        reference = self._reference_at(self.motion_time)
        limits = np.asarray(p['command_limits']['torso_rpy'], dtype=np.float32)
        self.torso_rpy = np.clip(authored_waist_to_torso_rpy(reference[7:][WAIST], limits)
                                 + self.command.torso_offset_rpy, -limits, limits).astype(np.float32)
        q, dq = state.qpos[7:], state.joint_velocity
        torques = np.zeros(29)
        lower_target = np.asarray(p['default_angles'], dtype=np.float32) + self.policy.action * p['action_scale']
        torques[LOWER] = (lower_target - q[LOWER]) * np.asarray(p['lower_kp']) - dq[LOWER] * np.asarray(p['lower_kd'])
        # Cancel the arm's gravity/Coriolis load so PD error is not needed merely to hold the pose.
        torques[ARMS] = ((self._arm_target() - q[ARMS]) * np.asarray(p['arm_kp']) - dq[ARMS] * np.asarray(p['arm_kd'])
                         + np.asarray(arm_bias))
        self._reference_for_update = reference
        return np.clip(torques, -self.effort_limits, self.effort_limits)

    def after_step(self, state: RobotState):
        """Advance time, update commands and run the policy at 50 Hz. Returns False once fallen."""
        p = self.p
        self.step_count += 1
        self.play_time += p['simulation_dt']
        if self.play_time >= p['upper_body_settle_seconds']:
            self.phase = 'playing' if self.motion_time < self.duration else 'finished'
            self.motion_time = min(self.duration, self.motion_time + p['simulation_dt'])
        if self.step_count % p['control_decimation'] == 0:
            self._update_auto_command(state, self._reference_for_update)
            observation = build_observation(state, self.policy.action, self.command, self.torso_rpy, p)
            self.policy.infer(observation, np.linalg.norm(self.command.nav) > .05)
        if self.step_count % RECORD_EVERY == 0:
            self.recording.append({'time': self.play_time, 'motion_time': self.motion_time, 'qpos': state.qpos.copy(),
                                   'nav': self.command.nav.copy(), 'height': self.command.height,
                                   'torso_rpy': self.torso_rpy.copy(), **self.tracking})
        tilt = np.linalg.norm(Rotation.from_quat(state.qpos[3:7][[1, 2, 3, 0]]).as_euler('xyz')[:2])
        if state.qpos[2] < .25 or tilt > np.deg2rad(45):
            self.phase = 'fallen'
            return False
        return True

    def _update_auto_command(self, state, reference):
        p = self.p
        limits = p['command_limits']
        if self.play_time < p['upper_body_settle_seconds']:
            self.command.nav[:] = 0
            self.command.height = float(np.clip(p['initial_height'] + reference[2] - self._reference_at(0.)[2],
                                                limits['minimum_height'], limits['maximum_height']))
            return
        control_dt = p['simulation_dt'] * p['control_decimation']
        following = self._reference_at(min(self.duration, self.motion_time + control_dt))
        nav, height, tracking = ghost_tracking_command(state.qpos[:7], reference[:7], following[:7], self.world_origin,
                                                       self._reference_at(0.)[:7], control_dt, p)
        alpha = float(p['auto_command_smoothing'])
        self.command.nav[:] = (1 - alpha) * self.command.nav + alpha * nav
        self.command.height = height
        self.tracking = tracking
        if self.motion_time >= self.duration and tracking['root_error_m'] < .01 and tracking['yaw_error_deg'] < 1.:
            self.command.nav[:] = 0


def tracking_summary(recording, reference):
    """Joint RMSE of recorded frames against the authored motion at the same motion time (degrees)."""
    playing = [frame for frame in recording if frame['time'] >= 0 and frame['motion_time'] > 0]
    if not playing:
        return {'lower_rmse_deg': 0., 'upper_rmse_deg': 0., 'upper_max_deg': 0.}
    reference = np.asarray(reference)
    errors = np.stack([frame['qpos'][7:] - reference[min(int(round(frame['motion_time'] * 50)), len(reference) - 1), 7:]
                       for frame in playing])
    return {'lower_rmse_deg': float(np.rad2deg(np.sqrt(np.mean(errors[:, LOWER] ** 2)))),
            'upper_rmse_deg': float(np.rad2deg(np.sqrt(np.mean(errors[:, ARMS] ** 2)))),
            'upper_max_deg': float(np.rad2deg(np.abs(errors[:, ARMS]).max())),
            'max_root_error_m': max(frame['root_error_m'] for frame in playing),
            'max_yaw_error_deg': max(frame['yaw_error_deg'] for frame in playing)}

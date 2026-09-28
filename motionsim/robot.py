"""Pinocchio FK and bounded, weighted whole-body IK. No dynamics stepping.

The IK objective, weights and acceptance rules are ported from Motion Creator
(`motioncreator/robot.py`). Only the kinematics backend changed: MuJoCo MJCF ->
Pinocchio URDF, so the editor and Isaac Sim read the same robot file.

qpos convention (project files, CSV, Motion Creator, Kimodo):
    [root xyz(3), root quaternion wxyz(4), 29 joint angles in URDF declaration order]
Pinocchio stores the root quaternion as xyzw and orders joints by tree traversal;
`Robot.to_pin` is the only place that converts between the two.
"""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
MODELS = json.loads((ROOT / 'integrations/robot-models.json').read_text())['models']
BASE_URDF = ROOT / MODELS['g1']['urdf']

HANDLES = {
    'pelvis': ('pelvis', (0, 0, 0), '골반'),
    'left_knee': ('left_knee_link', (0, 0, 0), '왼 무릎'),
    'right_knee': ('right_knee_link', (0, 0, 0), '오른 무릎'),
    'left_foot': ('left_ankle_roll_link', (.035, 0, -.035), '왼발'),
    'right_foot': ('right_ankle_roll_link', (.035, 0, -.035), '오른발'),
    'left_hand': ('left_wrist_yaw_link', (.055, 0, 0), '왼손'),
    'right_hand': ('right_wrist_yaw_link', (.055, 0, 0), '오른손'),
    'left_elbow': ('left_elbow_link', (0, 0, 0), '왼 팔꿈치'),
    'right_elbow': ('right_elbow_link', (0, 0, 0), '오른 팔꿈치'),
    'left_shoulder': ('left_shoulder_pitch_link', (0, 0, 0), '왼 어깨'),
    'right_shoulder': ('right_shoulder_pitch_link', (0, 0, 0), '오른 어깨'),
}
FEET = ('left_foot', 'right_foot')
ROTATABLE = ('pelvis', 'left_hand', 'right_hand', *FEET)
# Keyframe interpolation projects only these orientations (Motion Creator 2d5a32d): combined hip
# and waist controls are joint groups, so their orientation must not become an in-between target.
BASIC_ROTATABLE = ROTATABLE
HINGES = {key: key + '_joint' for key in ('left_elbow', 'right_elbow', 'left_knee', 'right_knee')}
BASIC_HANDLES = tuple(HANDLES)
PART_LABELS = {'hip_pitch': '고관절 피치', 'hip_roll': '고관절 롤', 'hip_yaw': '고관절 요',
               'knee': '무릎', 'ankle_pitch': '발목 피치', 'ankle_roll': '발목 롤',
               'shoulder_pitch': '어깨 피치', 'shoulder_roll': '어깨 롤', 'shoulder_yaw': '어깨 요',
               'elbow': '팔꿈치', 'wrist_roll': '손목 롤', 'wrist_pitch': '손목 피치', 'wrist_yaw': '손목 요',
               'waist_yaw': '허리 요', 'waist_roll': '허리 롤', 'waist_pitch': '허리 피치'}
# Foot sole corners in the ankle-roll link frame, relative to the foot handle point.
SOLE_CORNERS = ((-.085, -.03, 0), (-.085, .03, 0), (.085, -.03, 0), (.085, .03, 0))


def revolute_joints(urdf_path):
    """(joint name, child link, axis) for revolute joints in URDF declaration order."""
    joints = []
    for joint in ET.parse(urdf_path).getroot().findall('joint'):
        if joint.get('type') == 'revolute':
            axis = joint.find('axis')
            joints.append((joint.get('name'), joint.find('child').get('link'),
                           np.array([float(v) for v in (axis.get('xyz') if axis is not None else '1 0 0').split()])))
    return joints


# URDF joint origins coincide with the child link frame, so joint handles sit at the link origin.
JOINT_NAMES = [name for name, _, _ in revolute_joints(BASE_URDF)]
JOINT_HANDLES = {}
for _name, _child, _ in revolute_joints(BASE_URDF):
    _part = _name.removesuffix('_joint')
    _side = '왼 ' if _part.startswith('left_') else '오른 ' if _part.startswith('right_') else ''
    JOINT_HANDLES[_name] = (_child, (0, 0, 0), _side + PART_LABELS[_part.removeprefix('left_').removeprefix('right_')])
HANDLES.update(JOINT_HANDLES)
HINGES.update({name: name for name in JOINT_HANDLES})
HIP_HANDLES = ('left_hip', 'right_hip')
for _side, _key in zip(('left', 'right'), HIP_HANDLES):
    HANDLES[_key] = (JOINT_HANDLES[f'{_side}_hip_roll_joint'][0], (0, 0, 0), ('왼' if _side == 'left' else '오른') + ' 고관절')
ROTATABLE = (*ROTATABLE, *HIP_HANDLES)
for _key, _joint, _label in [('waist', 'waist_roll_joint', '허리'),
                             ('left_ankle', 'left_ankle_roll_joint', '왼 발목'),
                             ('right_ankle', 'right_ankle_roll_joint', '오른 발목')]:
    HANDLES[_key] = (JOINT_HANDLES[_joint][0], (0, 0, 0), _label)
# Ankles expose motor-angle targets, not an arbitrary 3-D orientation target.
ROTATABLE = (*ROTATABLE, 'waist')
ANGLE_LOCKABLE = tuple(dict.fromkeys((*ROTATABLE, *HINGES)))
# Handles whose orientation comes from a different link than their position.
ORIENTATION_LINKS = {'left_hip': 'left_hip_yaw_link', 'right_hip': 'right_hip_yaw_link', 'waist': 'torso_link'}
TOOL_LABELS = {'scoop': '주걱', 'end_support': '끝단 받침'}


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def right_jacobian(v):
    theta = np.linalg.norm(v)
    k = skew(v)
    if theta < 1e-5:
        return np.eye(3) - .5*k + k@k/6
    return np.eye(3) - (1-np.cos(theta))/theta**2*k + (theta-np.sin(theta))/theta**3*(k@k)


def left_jacobian_inverse(v):
    theta = np.linalg.norm(v)
    k = skew(v)
    coefficient = 1/12 if theta < 1e-5 else (1 - .5*theta/np.tan(theta/2))/theta**2
    return np.eye(3) - .5*k + coefficient*(k@k)


def quat_matrix(wxyz):
    return Rotation.from_quat(np.asarray(wxyz)[[1, 2, 3, 0]]).as_matrix()


def matrix_quat(matrix):
    return Rotation.from_matrix(matrix).as_quat()[[3, 0, 1, 2]]


def file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def urdf_visuals(urdf_path):
    """(link, origin SE3, mesh path, rgba) for every mesh visual in the URDF."""
    root = ET.parse(urdf_path).getroot()
    materials = {m.get('name'): m.find('color') for m in root.findall('material')}
    visuals = []
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            origin = visual.find('origin')
            xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            material = visual.find('material')
            color = None
            if material is not None:
                color = material.find('color') if material.find('color') is not None else materials.get(material.get('name'))
            rgba = [float(v) for v in color.get('rgba').split()] if color is not None else [.7, .7, .7, 1.]
            placement = pin.SE3(pin.rpy.rpyToMatrix(*rpy), np.array(xyz))
            visuals.append((link.get('name'), placement, Path(urdf_path).parent / mesh.get('filename'), rgba))
    return visuals


class Robot:
    def __init__(self, model_id='g1'):
        if model_id not in MODELS:
            raise ValueError(f'Unknown robot model: {model_id}')
        self.model_id = model_id
        self.urdf_path = ROOT / MODELS[model_id]['urdf']
        self.fingerprint = file_sha256(self.urdf_path)
        self.compatible_fingerprints = {self.fingerprint}
        if model_id == 'g1-tools':
            # The gripper model changes only fixed end-effector links. Its 29 actuated joints,
            # order and qpos layout are identical to base G1, so base-model projects are portable.
            self.compatible_fingerprints.add(file_sha256(BASE_URDF))
        model = self.model = pin.buildModelFromUrdf(str(self.urdf_path), pin.JointModelFreeFlyer())
        joints = revolute_joints(self.urdf_path)
        self.names = [name for name, _, _ in joints]
        if self.names != JOINT_NAMES:
            raise ValueError('Robot model joint order differs from the base G1 URDF')
        self.nq, self.nv = 7 + len(self.names), 6 + len(self.names)
        joint_ids = [model.getJointId(name) for name in self.names]
        self.pin_q_index = np.array([model.joints[i].idx_q for i in joint_ids])
        self.pin_v_index = np.array([model.joints[i].idx_v for i in joint_ids])
        self.joint_axes = {name: axis for name, _, axis in joints}
        self.joint_ids = dict(zip(self.names, joint_ids))

        self.handles = dict(HANDLES)
        if model_id == 'g1-tools':
            self.handles.update(self._tool_handles())
        self.point_frames, self.orientation_frames, self.handle_joints = {}, {}, {}
        for key, (link, offset, _) in self.handles.items():
            link_frame = model.frames[model.getFrameId(link)]
            placement = link_frame.placement * pin.SE3(np.eye(3), np.asarray(offset, dtype=float))
            frame = pin.Frame(f'handle:{key}', link_frame.parent, model.getFrameId(link), placement,
                              pin.FrameType.OP_FRAME)
            self.point_frames[key] = model.addFrame(frame)
            self.orientation_frames[key] = model.getFrameId(ORIENTATION_LINKS.get(key, link))
            self.handle_joints[key] = link_frame.parent
        self.visuals = [(model.getFrameId(link), placement, path, rgba)
                        for link, placement, path, rgba in urdf_visuals(self.urdf_path)]

        self.lower = np.r_[[-4, -4, .20], model.lowerPositionLimit[self.pin_q_index]]
        self.upper = np.r_[[4, 4, 1.5], model.upperPositionLimit[self.pin_q_index]]
        self.q_indices = np.r_[0:3, 7:self.nq]
        self.home = np.zeros(self.nq)
        self.home[3] = 1.
        for side in ('left', 'right'):
            for name, value in [('hip_pitch', -.12), ('knee', .24), ('ankle_pitch', -.12), ('elbow', .15)]:
                self.home[7 + self.names.index(f'{side}_{name}_joint')] = value
        d = self.data(self.home)
        self.home[2] -= min(self.point(d, k)[0][2] for k in FEET)

    def _tool_handles(self):
        """Hand handles move to each tool TCP; orientation stays the wrist-yaw link frame."""
        model = self.model
        handles = {}
        for side in ('left', 'right'):
            end_effector = MODELS[self.model_id]['end_effectors'][side]
            wrist = model.frames[model.getFrameId(f'{side}_wrist_yaw_link')]
            tcp = model.frames[model.getFrameId(end_effector)]
            if tcp.parent != wrist.parent:
                raise ValueError(f'{end_effector} must be rigidly attached to {side}_wrist_yaw_link')
            offset = (wrist.placement.inverse() * tcp.placement).translation
            tool = end_effector.removeprefix(f'{side}_').removesuffix('_tcp')
            label = ('왼손 ' if side == 'left' else '오른손 ') + TOOL_LABELS.get(tool, tool) + ' TCP'
            handles[f'{side}_hand'] = (f'{side}_wrist_yaw_link', tuple(offset), label)
        return handles

    def to_pin(self, q):
        pq = np.empty(self.model.nq)
        pq[0:3] = q[0:3]
        pq[3:7] = q[[4, 5, 6, 3]]
        pq[self.pin_q_index] = q[7:]
        return pq

    def validate_q(self, value):
        q = np.array(value, dtype=float, copy=True)
        if q.shape != (self.nq,) or not np.isfinite(q).all():
            raise ValueError(f'qpos에는 유한한 숫자 {self.nq}개가 필요합니다')
        norm = np.linalg.norm(q[3:7])
        if abs(norm - 1.) > 1e-4:
            raise ValueError('루트 quaternion(wxyz)이 정규화되어 있지 않습니다')
        q[3:7] /= norm
        x = q[self.q_indices]
        if np.any(x < self.lower - 1e-7) or np.any(x > self.upper + 1e-7):
            raise ValueError('자세가 관절 한계 또는 작업 공간을 벗어났습니다')
        return q.copy()

    def data(self, q):
        d = self.model.createData()
        pin.framesForwardKinematics(self.model, d, self.to_pin(q))
        return d

    def point(self, d, key):
        return d.oMf[self.point_frames[key]].translation.copy(), d.oMf[self.orientation_frames[key]].rotation.copy()

    def distance(self, a, b):
        def ancestors(joint):
            out = []
            while joint:
                out.append(joint)
                joint = int(self.model.parents[joint])
            return out + [0]
        aa, bb = ancestors(self.handle_joints[a]), ancestors(self.handle_joints[b])
        return min(i + bb.index(n) for i, n in enumerate(aa) if n in bb)

    def solve(self, q, anchor, focus=None, target=None, pins=FEET, resistance=1., mode='elastic', targets=None, max_nfev=50,
              posture_reference=None, posture_weight=.055, selected_targets=None, orientation_targets=None,
              joint_targets=None, angle_pins=()):
        q, anchor = self.validate_q(q), self.validate_q(anchor)
        selected_targets = dict(selected_targets or {})
        orientation_targets = dict(orientation_targets or {})
        joint_targets = dict(joint_targets or {})
        angle_pins = tuple(dict.fromkeys(angle_pins or ()))
        if focus is not None:
            selected_targets[focus] = target
        if any(k not in HANDLES for k in [*pins, *selected_targets, *orientation_targets]):
            raise ValueError('알 수 없는 핸들입니다')
        if set(selected_targets) & set(pins):
            raise ValueError('선택한 부위에 고정된 부위가 있습니다. 이동하려면 먼저 고정을 해제하세요.')
        if any(k not in ROTATABLE for k in orientation_targets):
            raise ValueError('방향 회전은 골반·손·발·통합 고관절·허리에서 지원합니다. 개별 관절과 통합 발목은 관절각 목표를 사용하세요.')
        if any(k not in ANGLE_LOCKABLE for k in angle_pins):
            raise ValueError('각도 고정을 지원하지 않는 부위입니다.')
        if set(orientation_targets) & set(angle_pins):
            raise ValueError('각도가 고정된 부위를 회전하려면 각도 고정을 해제하세요.')
        if set(orientation_targets) & set(pins) & set(FEET):
            raise ValueError('발 방향이 고정되어 있습니다. 회전하려면 발 고정을 해제하세요.')
        if any(k not in self.names for k in joint_targets):
            raise ValueError('알 수 없는 관절입니다')
        for key, value in joint_targets.items():
            i = self.names.index(key)
            if not np.isfinite(value) or not self.lower[3+i] <= value <= self.upper[3+i]:
                raise ValueError('관절 목표가 관절 한계를 벗어났습니다')
        ad = self.data(anchor)
        base_targets = {k: self.point(ad, k) for k in HANDLES}
        orientation_locks = {key: base_targets[key][1] for key in angle_pins if key in ROTATABLE}
        joint_locks = {HINGES[key]: float(anchor[7 + self.names.index(HINGES[key])])
                       for key in angle_pins if key in HINGES}
        if set(joint_targets) & set(joint_locks):
            raise ValueError('각도가 고정된 관절을 조정하려면 각도 고정을 해제하세요.')
        desired = {k: (p.copy(), r.copy()) for k, (p, r) in base_targets.items()}
        if targets:
            desired.update(targets)
        for key, value in selected_targets.items():
            t = np.asarray(value, dtype=float)
            if t.shape != (3,) or not np.isfinite(t).all() or np.max(np.abs(t)) > 5:
                raise ValueError('목표 위치는 5 m 이내의 유한한 XYZ여야 합니다')
            desired[key] = (t, desired[key][1])
        for key, value in orientation_targets.items():
            quat = np.asarray(value, dtype=float)
            if quat.shape != (4,) or not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1) > 1e-4:
                raise ValueError('방향 목표는 정규화된 xyzw quaternion이어야 합니다')
            desired[key] = (desired[key][0], Rotation.from_quat(quat).as_matrix())
        for key, rotation in orientation_locks.items():
            desired[key] = (desired[key][0], rotation)
        active = set(selected_targets) | set(orientation_targets) | set(joint_targets) | set(angle_pins)
        # Extra selectable anchors must not add passive resistance everywhere.
        solve_handles = tuple(dict.fromkeys([*BASIC_HANDLES, *pins, *selected_targets,
                                             *orientation_targets, *orientation_locks]))
        weights = {}
        for k in solve_handles:
            if k in pins:
                weights[k] = 180.
                desired[k] = (base_targets[k][0], desired[k][1] if k in orientation_targets else base_targets[k][1])
            elif k in selected_targets or k in orientation_targets:
                weights[k] = 28.
            elif k in orientation_locks:
                # An angle-only lock must not resist translation of the handle.
                weights[k] = 0.
            elif targets:
                weights[k] = 4.
            elif mode == 'elastic':
                distance = min(self.distance(key, k) for key in (active or {'pelvis'}))
                weights[k] = resistance * (.12 + 3.0 * min(distance / 10, 1) ** 2)
            else:
                weights[k] = .015
        model, d = self.model, self.model.createData()
        rotate_base = 'pelvis' in orientation_targets
        n_basic = len(self.q_indices)
        n = n_basic + (3 if rotate_base else 0)
        base_rotation = quat_matrix(q[3:7])
        posture_q = anchor if posture_reference is None else self.validate_q(posture_reference)
        posture_x = posture_q[self.q_indices]
        if rotate_base:
            posture_x = np.r_[posture_x, Rotation.from_matrix(base_rotation.T @ quat_matrix(posture_q[3:7])).as_rotvec()]
        joint_columns = self.pin_v_index
        rotation_map = np.eye(3)

        def frame_jacobian(frame):
            """6 x n Jacobian [linear; angular] (world-aligned) w.r.t. the optimisation variables."""
            full = pin.getFrameJacobian(model, d, frame, pin.LOCAL_WORLD_ALIGNED)
            jacobian = np.zeros((6, n))
            # Root translation variables are world-frame position offsets.
            jacobian[:3, :3] = np.eye(3)
            jacobian[:, 3:n_basic] = full[:, joint_columns]
            if rotate_base:
                # Pinocchio's free-flyer angular velocity is expressed in the root frame, like MuJoCo's.
                jacobian[:, n_basic:] = full[:, 3:6] @ rotation_map
            return jacobian

        def evaluate(x, jac=False):
            nonlocal rotation_map
            current = q.copy()
            current[self.q_indices] = x[:n_basic]
            if rotate_base:
                current[3:7] = matrix_quat(base_rotation @ Rotation.from_rotvec(x[-3:]).as_matrix())
                rotation_map = right_jacobian(x[-3:])
            if jac:
                pin.computeJointJacobians(model, d, self.to_pin(current))
                pin.updateFramePlacements(model, d)
            else:
                pin.framesForwardKinematics(model, d, self.to_pin(current))
            residuals, matrices = [], []
            for k in solve_handles:
                p, r = self.point(d, k)
                tp, tr = desired[k]
                jpos = jrot = None
                if jac:
                    jpos = frame_jacobian(self.point_frames[k])
                    jrot = frame_jacobian(self.orientation_frames[k])[3:]
                if weights[k]:
                    residuals.append(weights[k] * (p-tp))
                    if jac:
                        matrices.append(weights[k] * jpos[:3])
                if k in orientation_targets or k in orientation_locks or (k in pins and k in FEET):
                    w = 90. if k in orientation_locks or (k in pins and k in FEET) else 18.
                    err = Rotation.from_matrix(r @ tr.T).as_rotvec()
                    residuals.append(w * err)
                    if jac:
                        matrices.append(w * left_jacobian_inverse(err) @ jrot)
                if k in FEET:
                    for offset in SOLE_CORNERS:
                        corner = p + r @ offset
                        residuals.append(np.array([120 * min(0., corner[2])]))
                        if jac:
                            # Velocity of a body point: v_p + w x (corner - p).
                            corner_jacobian = jpos[:3] - skew(corner - p) @ jpos[3:]
                            matrices.append(120*corner_jacobian[2:3] if corner[2] < 0 else np.zeros((1, n)))
            for key, value in joint_targets.items():
                index = 3 + self.names.index(key)
                residuals.append(np.array([24.*(x[index]-value)]))
                if jac:
                    row = np.zeros((1, n)); row[0, index] = 24.; matrices.append(row)
            for key, value in joint_locks.items():
                index = 3 + self.names.index(key)
                residuals.append(np.array([120.*(x[index]-value)]))
                if jac:
                    row = np.zeros((1, n)); row[0, index] = 120.; matrices.append(row)
            residuals.append(posture_weight*(x-posture_x))
            if jac:
                matrices.append(posture_weight*np.eye(n))
                return np.vstack(matrices)
            return np.concatenate(residuals)

        initial = q[self.q_indices]
        lower, upper = self.lower, self.upper
        if rotate_base:
            initial = np.r_[initial, np.zeros(3)]
            lower, upper = np.r_[lower, [-np.pi]*3], np.r_[upper, [np.pi]*3]
        result = least_squares(evaluate, np.clip(initial, lower+1e-9, upper-1e-9),
                               jac=lambda x: evaluate(x, True), bounds=(lower, upper),
                               max_nfev=max_nfev, ftol=1e-5, xtol=1e-6, gtol=1e-5)
        answer = q.copy()
        answer[self.q_indices] = result.x[:n_basic]
        if rotate_base:
            answer[3:7] = matrix_quat(base_rotation @ Rotation.from_rotvec(result.x[-3:]).as_matrix())
        rd = self.data(answer)
        pin_error = max((np.linalg.norm(self.point(rd, k)[0]-base_targets[k][0]) for k in pins), default=0.)
        angle_error = max((np.linalg.norm(Rotation.from_matrix(self.point(rd, k)[1] @ base_targets[k][1].T).as_rotvec()) for k in pins if k in FEET), default=0.)
        orientation_pin_error = max((np.linalg.norm(Rotation.from_matrix(self.point(rd, k)[1] @ base_targets[k][1].T).as_rotvec())
                                     for k in orientation_locks), default=0.)
        joint_pin_error = max((abs(answer[7 + self.names.index(name)] - value) for name, value in joint_locks.items()), default=0.)
        angle_pin_error = max(orientation_pin_error, joint_pin_error)
        rejected = bool(pin_error > .003 or angle_error > .015 or angle_pin_error > np.deg2rad(.5))
        if rejected:
            answer = q; rd = self.data(answer)
        errors = {key: float(np.linalg.norm(self.point(rd, key)[0]-desired[key][0]))*1000
                  for key in set(selected_targets) | set(orientation_targets)}
        rotation_errors = {key: float(Rotation.from_matrix(self.point(rd, key)[1] @ desired[key][1].T).magnitude()*180/np.pi) for key in orientation_targets}
        joint_errors = {key: float(abs(answer[7+self.names.index(key)]-value)*180/np.pi) for key, value in joint_targets.items()}
        angular_error = max([*rotation_errors.values(), *joint_errors.values()], default=0.)
        error = max(errors.values(), default=0.)
        return answer, {'target_error_mm': error, 'pin_error_mm': float(pin_error*1000),
                        'rejected': rejected, 'converged': error < 10 and angular_error < 2 and not rejected,
                        'evaluations': result.nfev, 'target_errors_mm': errors,
                        'angle_error_deg': angular_error, 'rotation_errors_deg': rotation_errors,
                        'joint_errors_deg': joint_errors,
                        'angle_pin_error_deg': float(np.rad2deg(angle_pin_error))}

    def state(self, q):
        """Handle poses (wxyz), visual link poses, COM, sole clearance and hinge axes for the viewer."""
        d = self.data(q)
        handles = {k: {'position': p, 'wxyz': matrix_quat(r), 'label': self.handles[k][2]}
                   for k in HANDLES for p, r in [self.point(d, k)]}
        visuals = []
        for frame, placement, _, _ in self.visuals:
            pose = d.oMf[frame] * placement
            visuals.append((pose.translation.copy(), matrix_quat(pose.rotation)))
        com = pin.centerOfMass(self.model, d, self.to_pin(q))
        floor_min = min(self.point(d, k)[0][2] + (self.point(d, k)[1] @ np.array(o))[2] for k in FEET for o in SOLE_CORNERS)
        hinges = {}
        for key, name in HINGES.items():
            placement = d.oMi[self.joint_ids[name]]
            index = self.names.index(name)
            hinges[key] = {'joint_name': name, 'angle': float(q[7 + index]),
                           'limits': (float(self.lower[3 + index]), float(self.upper[3 + index])),
                           'axis_world': placement.rotation @ self.joint_axes[name], 'position': placement.translation.copy()}
        return {'model_id': self.model_id, 'qpos': np.asarray(q), 'handles': handles, 'visuals': visuals,
                'com': np.asarray(com).copy(), 'floor_min_mm': float(floor_min * 1000), 'hinges': hinges}

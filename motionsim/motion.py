"""Project contract, keyframe interpolation and atomic save bundles.

Keyframe semantics follow Motion Creator: `duration` is the travel time from the previous
keyframe; the first keyframe's duration is unused. Interpolation uses quintic easing,
shortest-path root SLERP, and re-projects handles that stay pinned across a segment.
"""
from datetime import datetime, timezone
from pathlib import Path
import io
import json
import os
import re
import uuid

import numpy as np
from scipy.spatial.transform import Rotation

from .robot import ANGLE_LOCKABLE, BASIC_ROTATABLE as ROTATABLE, FEET, HANDLES, HINGES, ROOT, Robot, matrix_quat, quat_matrix

FORMAT = 'motionsim.g1.v1'
MOTIONCREATOR_FORMAT = 'motioncreator.g1.v1'
COORDINATE_SYSTEM = 'right-handed, +X forward, +Y left, +Z up'
UNITS = {'position': 'm', 'angle': 'rad', 'time': 's'}
MAX_KEYFRAMES = 100
MAX_PROJECT_SECONDS = 600
MIN_DURATION, MAX_DURATION = .1, 60.
PROJECT_ID_PATTERN = re.compile(r'[0-9a-f]{32}')
DEFAULT_MOTIONS_DIR = ROOT / 'motions'


def _created_now():
    return datetime.now(timezone.utc).astimezone().isoformat(timespec='seconds')


def new_project(robot: Robot, name=''):
    return {'format': FORMAT, 'project_id': uuid.uuid4().hex, 'created_at': _created_now(), 'name': name,
            'model_id': robot.model_id, 'model_sha256': robot.fingerprint, 'joint_names': list(robot.names),
            'coordinate_system': COORDINATE_SYSTEM, 'units': dict(UNITS),
            'keyframes': [{'name': 'Stand', 'duration': 2., 'qpos': robot.home.tolist(),
                           'pins': list(FEET), 'angle_pins': []}],
            'current_qpos': robot.home.tolist(), 'pins': list(FEET), 'angle_pins': []}


def validate_project(robot: Robot, project):
    """Validate in place. Accepts projects authored on a compatible model and rebinds them to `robot`."""
    if project.get('format') != FORMAT:
        raise ValueError(f'프로젝트 형식이 {FORMAT}이 아닙니다')
    if project.get('joint_names') != robot.names:
        raise ValueError('관절 순서가 현재 모델과 다릅니다')
    if project.get('model_sha256') not in robot.compatible_fingerprints:
        raise ValueError('프로젝트의 로봇 모델이 현재 모델과 호환되지 않습니다')
    if not PROJECT_ID_PATTERN.fullmatch(str(project.get('project_id', ''))):
        raise ValueError('project_id는 소문자 hex 32자여야 합니다')
    if not isinstance(project.get('name'), str) or len(project['name']) > 80:
        raise ValueError('프로젝트 이름은 80자 이하의 문자열이어야 합니다')
    frames = project.get('keyframes')
    if not isinstance(frames, list) or not 1 <= len(frames) <= MAX_KEYFRAMES:
        raise ValueError(f'키프레임은 1–{MAX_KEYFRAMES}개여야 합니다')
    total = 0.
    for index, frame in enumerate(frames):
        if not isinstance(frame.get('name'), str):
            raise ValueError('키프레임 이름은 문자열이어야 합니다')
        frame['qpos'] = robot.validate_q(frame.get('qpos')).tolist()
        duration = float(frame.get('duration', 2.))
        if not np.isfinite(duration) or not MIN_DURATION <= duration <= MAX_DURATION:
            raise ValueError(f'키프레임 duration은 {MIN_DURATION:g}–{MAX_DURATION:g}초여야 합니다')
        if index:
            total += duration
        _validate_pins(frame)
    if total > MAX_PROJECT_SECONDS:
        raise ValueError(f'프로젝트 길이는 최대 {MAX_PROJECT_SECONDS}초입니다')
    if project.get('current_qpos') is not None:
        project['current_qpos'] = robot.validate_q(project['current_qpos']).tolist()
    _validate_pins(project)
    project['model_id'] = robot.model_id
    project['model_sha256'] = robot.fingerprint
    return project


def _validate_pins(item):
    item.setdefault('pins', [])
    item.setdefault('angle_pins', [])
    if any(k not in HANDLES for k in item['pins']):
        raise ValueError('알 수 없는 고정 핸들입니다')
    if any(k not in ANGLE_LOCKABLE for k in item['angle_pins']):
        raise ValueError('각도 고정을 지원하지 않는 핸들입니다')


def import_motioncreator_project(robot: Robot, source):
    """Convert a Motion Creator project. Returns (project, warnings); keyframes only."""
    if source.get('format') != MOTIONCREATOR_FORMAT:
        raise ValueError('Motion Creator 프로젝트(motioncreator.g1.v1)가 아닙니다')
    if source.get('joint_names') != robot.names:
        raise ValueError('Motion Creator 프로젝트의 관절 순서가 현재 모델과 다릅니다')
    frames = source.get('keyframes', [])
    if any(frame.get('samples') is not None for frame in frames):
        raise ValueError('가져온 NPZ/CSV 클립 프로젝트는 아직 지원하지 않습니다')
    project = new_project(robot, str(source.get('name', ''))[:80])
    project['keyframes'] = [{'name': frame.get('name', ''), 'duration': float(frame.get('duration', 2.)),
                             'qpos': frame['qpos'], 'pins': list(frame.get('pins', [])),
                             'angle_pins': list(frame.get('angle_pins', []))} for frame in frames]
    project['current_qpos'] = source.get('current_qpos', frames[0]['qpos'] if frames else robot.home.tolist())
    project['pins'] = list(source.get('pins', FEET))
    project['angle_pins'] = list(source.get('angle_pins', []))
    project['source'] = {'format': MOTIONCREATOR_FORMAT, 'project_id': source.get('project_id'),
                         'model_sha256': source.get('model_sha256')}
    warnings = []
    ignored = [key for key in ('scene_objects', 'scene_groups', 'box') if source.get(key)]
    if ignored:
        warnings.append('장면 정보는 아직 지원하지 않아 무시했습니다: ' + ', '.join(ignored))
    if any(frame.get('grasp') for frame in frames):
        warnings.append('키프레임 파지 설정은 아직 지원하지 않아 무시했습니다')
    warnings.append('Motion Creator(MJCF)와 motion-sim(URDF)의 기구학 차이로 고정 핸들 위치가 미세하게 다를 수 있습니다')
    return validate_project(robot, project), warnings


def keyframe_times(project):
    times, elapsed = [], 0.
    for index, frame in enumerate(project['keyframes']):
        if index:
            elapsed += float(frame['duration'])
        times.append(elapsed)
    return times


def motion_result(robot: Robot, qpos, times, contacts, fps, pin_errors=()):
    """qvel layout follows MuJoCo's free joint: root linear velocity in world, angular velocity in the root frame."""
    qpos, times = np.asarray(qpos), np.asarray(times)
    qvel = np.zeros((len(qpos), robot.nv))
    for index in range(len(qpos)):
        lo, hi = max(0, index - 1), min(len(qpos) - 1, index + 1)
        if hi > lo:
            dt = times[hi] - times[lo]
            qvel[index, :3] = (qpos[hi, :3] - qpos[lo, :3]) / dt
            qvel[index, 3:6] = Rotation.from_matrix(quat_matrix(qpos[lo, 3:7]).T @ quat_matrix(qpos[hi, 3:7])).as_rotvec() / dt
            qvel[index, 6:] = (qpos[hi, 7:] - qpos[lo, 7:]) / dt
    qacc = np.gradient(qvel, times, axis=0) if len(times) > 1 else np.zeros_like(qvel)
    return {'time': times, 'qpos': qpos, 'qvel': qvel, 'qacc': qacc, 'contacts': np.asarray(contacts, dtype=bool),
            'fps': fps, 'max_pin_error_mm': max(pin_errors, default=0.)}


def compile_motion(robot: Robot, project, fps=30):
    validate_project(robot, project)
    if not 1 <= fps <= 120:
        raise ValueError('FPS는 1–120이어야 합니다')
    frames = project['keyframes']
    poses, times, contacts = [robot.validate_q(frames[0]['qpos'])], [0.], [[k in frames[0].get('pins', []) for k in FEET]]
    pin_errors = []
    elapsed = 0.
    for number, (a, b) in enumerate(zip(frames, frames[1:]), start=2):
        qa, qb = robot.validate_q(a['qpos']), robot.validate_q(b['qpos'])
        if np.dot(qa[3:7], poses[-1][3:7]) < 0:
            qa[3:7] *= -1
        if np.dot(qa[3:7], qb[3:7]) < 0:
            qb[3:7] *= -1
        da, db = robot.data(qa), robot.data(qb)
        pa = {k: robot.point(da, k) for k in HANDLES}
        pb = {k: robot.point(db, k) for k in HANDLES}
        pins = sorted(set(a.get('pins', [])) & set(b.get('pins', [])))
        angle_pins = sorted(set(a.get('angle_pins', [])) & set(b.get('angle_pins', [])))
        where = f'{number - 1}→{number}번 키프레임 구간: '
        for k in pins:
            if np.linalg.norm(pa[k][0] - pb[k][0]) > .004:
                raise ValueError(where + f'{HANDLES[k][2]}이(가) 고정 상태에서 위치가 바뀝니다. 한쪽 키프레임에서 고정을 해제하세요.')
            if k in FEET and np.linalg.norm(pa[k][1] - pb[k][1]) > .02:
                raise ValueError(where + f'{HANDLES[k][2]}이(가) 고정 상태에서 방향이 바뀝니다')
        for k in angle_pins:
            if k in HINGES:
                index = 7 + robot.names.index(HINGES[k])
                if abs(qa[index] - qb[index]) > np.deg2rad(.5):
                    raise ValueError(where + f'{HANDLES[k][2]}이(가) 각도 고정 상태에서 관절각이 바뀝니다')
            elif np.linalg.norm(Rotation.from_matrix(pa[k][1].T @ pb[k][1]).as_rotvec()) > np.deg2rad(.5):
                raise ValueError(where + f'{HANDLES[k][2]}이(가) 각도 고정 상태에서 방향이 바뀝니다')
        # Each destination frame's duration describes travel time from its predecessor.
        count = max(1, round(float(b['duration']) * fps))
        duration = count / fps
        root_rotation = quat_matrix(qa[3:7])
        root_delta = Rotation.from_matrix(root_rotation.T @ quat_matrix(qb[3:7])).as_rotvec()
        angular_deltas = {k: Rotation.from_matrix(pa[k][1].T @ pb[k][1]).as_rotvec() for k in ROTATABLE}
        for i in range(1, count + 1):
            u = i / count
            s = u*u*u*(10 + u*(-15 + 6*u))  # quintic easing, zero endpoint velocity/acceleration
            q = (1-s)*qa + s*qb
            q[3:7] = matrix_quat(root_rotation @ Rotation.from_rotvec(s*root_delta).as_matrix())
            if np.dot(q[3:7], qa[3:7]) < 0:
                q[3:7] *= -1
            if i == count:
                q = qb.copy()
            if i != count and (pins or angle_pins):
                rotations = {k: pa[k][1] @ Rotation.from_rotvec(s*angular_deltas[k]).as_matrix() for k in ROTATABLE}
                targets = {k: ((1-s)*pa[k][0] + s*pb[k][0], rotations.get(k, pa[k][1])) for k in HANDLES}
                # Root orientation follows SLERP exactly; only end-effector rotations need projection.
                orientations = {k: Rotation.from_matrix(rotations[k]).as_quat() for k in ROTATABLE
                                if k != 'pelvis' and k not in angle_pins and not (k in FEET and k in pins)
                                and np.linalg.norm(angular_deltas[k]) > 1e-5}
                q, info = robot.solve(q, qa, pins=pins, targets=targets, max_nfev=18,
                                      posture_reference=q, posture_weight=.8, orientation_targets=orientations,
                                      angle_pins=angle_pins)
                if info['rejected']:
                    raise ValueError(where + '고정을 유지할 수 없습니다. 중간 키프레임을 추가하거나 고정을 해제하세요.')
                pin_errors.append(info['pin_error_mm'])
            poses.append(q)
            times.append(elapsed + i / fps)
            contacts.append([k in pins for k in FEET])
        elapsed += duration
    return motion_result(robot, poses, times, contacts, fps, pin_errors)


def _atomic_bytes(path, content):
    path = Path(path)
    temporary = path.with_name(f'.{path.name}.{uuid.uuid4().hex}.tmp')
    try:
        temporary.write_bytes(content)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _folder_name(project):
    name = re.sub(r'[^\w-]', '_', project['name'].strip(), flags=re.UNICODE).strip('_')[:60] or 'motion'
    return f'{name}_{project["project_id"][:8]}'


def _existing_project_folder(root_folder, project_id):
    for path in Path(root_folder).glob('*/project.json'):
        try:
            if json.loads(path.read_text()).get('project_id') == project_id:
                return path.parent
        except (OSError, ValueError):
            continue
    return None


def save_bundle(robot: Robot, project, fps=30, directory=None, save_as=False):
    """Create or atomically update one project folder: project.json, motion.csv, metadata.json."""
    project = json.loads(json.dumps(project))
    if save_as:
        project['project_id'] = uuid.uuid4().hex
        project['created_at'] = _created_now()
    motion = compile_motion(robot, project, fps)
    root_folder = Path(directory or DEFAULT_MOTIONS_DIR)
    root_folder.mkdir(parents=True, exist_ok=True)
    desired = root_folder / _folder_name(project)
    folder = _existing_project_folder(root_folder, project['project_id'])
    if folder is None:
        folder = desired
        folder.mkdir()
    elif folder != desired:
        if desired.exists():
            raise ValueError('같은 이름의 다른 프로젝트 폴더가 이미 있습니다')
        folder.rename(desired)
        folder = desired
    floor = [robot.state(q)['floor_min_mm'] for q in motion['qpos']]
    metadata = {'format': FORMAT, 'model_id': robot.model_id, 'model_sha256': robot.fingerprint,
                'joint_names': robot.names, 'project_id': project['project_id'], 'name': project['name'],
                'saved_at': _created_now(), 'fps': fps, 'samples': len(motion['time']),
                'duration_s': float(motion['time'][-1]),
                'csv_columns': 'qpos: root xyz(3), root quaternion wxyz(4), 29 joint angles (joint_names order); no header',
                'coordinate_system': COORDINATE_SYSTEM, 'units': UNITS,
                'trajectory_type': 'kinematic reference',
                'interpolation': 'quintic easing, shortest-path root SLERP, orientation-aware IK for shared pins',
                'validation': {'kind': 'kinematic only', 'max_pin_error_mm': motion['max_pin_error_mm'],
                               'minimum_sole_height_mm': min(floor),
                               'max_joint_speed_rad_s': float(np.abs(motion['qvel'][:, 6:]).max()),
                               'self_collision_checked': False, 'dynamic_balance_checked': False}}
    csv_buffer = io.StringIO()
    np.savetxt(csv_buffer, motion['qpos'], delimiter=',')
    _atomic_bytes(folder / 'motion.csv', csv_buffer.getvalue().encode())
    _atomic_bytes(folder / 'metadata.json', json.dumps(metadata, ensure_ascii=False, indent=2).encode())
    _atomic_bytes(folder / 'project.json', json.dumps(project, ensure_ascii=False, indent=2).encode())
    return folder, project


def list_saved(directory=None):
    """Saved projects, newest first: (folder, project name)."""
    entries = []
    for path in Path(directory or DEFAULT_MOTIONS_DIR).glob('*/project.json'):
        try:
            entries.append((path.stat().st_mtime, path.parent, json.loads(path.read_text()).get('name', '')))
        except (OSError, ValueError):
            continue
    return [(folder, name) for _, folder, name in sorted(entries, key=lambda e: e[0], reverse=True)]


def load_project(robot: Robot, content):
    """Parse project JSON text or dict. Returns (project, warnings)."""
    source = json.loads(content) if isinstance(content, (str, bytes)) else content
    if source.get('format') == MOTIONCREATOR_FORMAT:
        return import_motioncreator_project(robot, source)
    return validate_project(robot, source), []

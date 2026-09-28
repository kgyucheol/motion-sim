"""Motion Creator(MuJoCo)가 만든 기준값과 motion-sim(Pinocchio)의 FK·IK·보간 결과를 비교한다.

fixture 생성: tests/fixtures/make_motioncreator_reference.py (Motion Creator 환경에서 실행)
"""
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from motionsim.motion import compile_motion, new_project
from motionsim.robot import HANDLES, Robot

FIXTURES = Path(__file__).parent / 'fixtures'
REFERENCE = np.load(FIXTURES / 'motioncreator_reference.npz')
META = json.loads((FIXTURES / 'motioncreator_reference.json').read_text())
MODEL_IDS = ('g1', 'g1-tools')


@pytest.fixture(scope='module', params=MODEL_IDS)
def robot(request):
    return Robot(request.param)


def reference(robot, name):
    return REFERENCE[f"{robot.model_id.replace('-', '_')}_{name}"]


def test_joint_order_and_handles_match(robot):
    meta = META['models'][robot.model_id]
    assert robot.names == meta['joint_names']
    assert sorted(HANDLES) == meta['handles']


def test_home_pose_matches(robot):
    np.testing.assert_allclose(robot.home, reference(robot, 'home'), atol=1e-6)


def test_forward_kinematics_matches(robot):
    names = sorted(HANDLES)
    worst_position, worst_angle = 0., 0.
    for q, positions, rotations in zip(reference(robot, 'fk_q'), reference(robot, 'fk_pos'), reference(robot, 'fk_rot')):
        d = robot.data(q)
        for name, position, rotation in zip(names, positions, rotations):
            p, r = robot.point(d, name)
            worst_position = max(worst_position, np.linalg.norm(p - position))
            worst_angle = max(worst_angle, Rotation.from_matrix(r @ rotation.T).magnitude())
    assert worst_position < 1e-5, f'handle position differs by {worst_position * 1000:.4f} mm'
    assert worst_angle < 1e-5, f'handle orientation differs by {np.rad2deg(worst_angle):.5f} deg'


@pytest.mark.parametrize('index', range(5))
def test_inverse_kinematics_matches(robot, index):
    case = META['models'][robot.model_id]['ik'][index]
    q, info = robot.solve(robot.home, robot.home, **case['kwargs'])
    expected = np.asarray(case['q'])
    assert info['rejected'] == case['info']['rejected']
    # Same objective and optimiser. FK differs from MuJoCo only at ~1e-8, which moves free-mode
    # solutions (weak posture term) by up to ~3e-4 rad (measured 2026-09-28).
    assert np.abs(q - expected).max() < 1e-3, f"{case['name']}: max qpos diff {np.abs(q - expected).max():.2e}"
    assert abs(info['target_error_mm'] - case['info']['target_error_mm']) < .5
    assert info['pin_error_mm'] < 3.


def test_compiled_motion_matches(robot):
    meta = META['models'][robot.model_id]
    project = new_project(robot, 'fixture')
    project['keyframes'] = meta['motion_keyframes']
    motion = compile_motion(robot, project, 30)
    np.testing.assert_allclose(motion['time'], reference(robot, 'motion_time'), atol=1e-9)
    # Measured difference is ~2e-7 rad. Interpolation must project only BASIC_ROTATABLE orientations.
    assert np.abs(motion['qpos'] - reference(robot, 'motion_qpos')).max() < 1e-5
    assert motion['max_pin_error_mm'] < 3.

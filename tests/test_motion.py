"""프로젝트 계약, Motion Creator 가져오기, 보간, 저장 번들 테스트."""
import json

import numpy as np
import pytest

from motionsim.motion import (FORMAT, compile_motion, import_motioncreator_project, keyframe_times, list_saved,
                              load_project, new_project, save_bundle, validate_project)
from motionsim.robot import FEET, Robot


@pytest.fixture(scope='module')
def robot():
    return Robot('g1')


@pytest.fixture(scope='module')
def reach(robot):
    target = robot.point(robot.data(robot.home), 'right_hand')[0] + [.08, 0, .06]
    q, info = robot.solve(robot.home, robot.home, selected_targets={'right_hand': target.tolist()})
    assert info['converged']
    return q


def two_keyframe_project(robot, reach, duration=1.):
    project = new_project(robot, 'test')
    project['keyframes'].append({'name': 'reach', 'duration': duration, 'qpos': reach.tolist(),
                                 'pins': list(FEET), 'angle_pins': []})
    return project


def test_new_project_is_valid(robot):
    project = validate_project(robot, new_project(robot))
    assert project['format'] == FORMAT
    assert project['model_id'] == 'g1' and project['model_sha256'] == robot.fingerprint
    assert project['joint_names'] == robot.names


def test_home_stands_on_floor(robot):
    state = robot.state(robot.home)
    assert abs(state['floor_min_mm']) < .5
    assert len(state['visuals']) == len(robot.visuals)


@pytest.mark.parametrize('field, value, message', [
    ('joint_names', ['wrong'], '관절 순서'),
    ('model_sha256', '0' * 64, '호환되지'),
    ('project_id', 'not-hex', 'project_id'),
])
def test_validate_rejects_incompatible_projects(robot, field, value, message):
    project = new_project(robot)
    project[field] = value
    with pytest.raises(ValueError, match=message):
        validate_project(robot, project)


def test_validate_rejects_bad_keyframes(robot, reach):
    project = two_keyframe_project(robot, reach, duration=.05)
    with pytest.raises(ValueError, match='duration'):
        validate_project(robot, project)
    project = two_keyframe_project(robot, reach)
    project['keyframes'][1]['qpos'][3] = 2.
    with pytest.raises(ValueError, match='quaternion'):
        validate_project(robot, project)
    project = two_keyframe_project(robot, reach)
    project['keyframes'][1]['pins'] = ['nose']
    with pytest.raises(ValueError, match='고정 핸들'):
        validate_project(robot, project)


def test_tools_model_opens_base_projects():
    tools = Robot('g1-tools')
    project = validate_project(tools, new_project(Robot('g1')))
    assert project['model_id'] == 'g1-tools' and project['model_sha256'] == tools.fingerprint


def test_compile_keeps_feet_and_timing(robot, reach):
    motion = compile_motion(robot, two_keyframe_project(robot, reach, duration=1.), fps=30)
    assert len(motion['time']) == 31 and motion['time'][-1] == pytest.approx(1.)
    np.testing.assert_allclose(motion['qpos'][0], robot.home)
    np.testing.assert_allclose(motion['qpos'][-1], reach)
    home_feet = [robot.point(robot.data(robot.home), k)[0] for k in FEET]
    for q in motion['qpos']:
        d = robot.data(q)
        for key, reference in zip(FEET, home_feet):
            assert np.linalg.norm(robot.point(d, key)[0] - reference) < .003
    assert motion['contacts'].all()


def test_compile_rejects_moving_pinned_foot(robot):
    target = robot.point(robot.data(robot.home), 'left_foot')[0] + [.1, 0, .05]
    step, _ = robot.solve(robot.home, robot.home, pins=('right_foot',), selected_targets={'left_foot': target.tolist()})
    project = new_project(robot)
    project['keyframes'].append({'name': 'step', 'duration': 1., 'qpos': step.tolist(), 'pins': list(FEET), 'angle_pins': []})
    with pytest.raises(ValueError, match='고정 상태에서 위치'):
        compile_motion(robot, project)


def test_keyframe_times_ignore_first_duration(robot, reach):
    project = two_keyframe_project(robot, reach, duration=1.5)
    project['keyframes'][0]['duration'] = 9.
    assert keyframe_times(project) == [0., 1.5]


def test_import_motioncreator_project(robot, reach):
    source = {'format': 'motioncreator.g1.v1', 'name': 'mc', 'project_id': 'a' * 32, 'model_sha256': 'b' * 64,
              'joint_names': robot.names,
              'scene_objects': [{'id': 'legacy-box', 'name': '박스 1', 'shape': 'box', 'position': [.4, 0, .12],
                                 'quaternion_xyzw': [0, 0, .70710678, .70710678], 'size': [.3, .5, .24], 'mass_kg': 1,
                                 'friction': .7, 'color': '#ff8400', 'opacity': .5, 'visible': True},
                                {'id': 'glb', 'name': '스캔', 'shape': 'box', 'position': [0, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1],
                                 'size': [.1, .1, .1], 'asset_id': 'f' * 64}],
              'keyframes': [{'name': 'Stand', 'duration': 2., 'qpos': robot.home.tolist(), 'pins': list(FEET)},
                            {'name': 'reach', 'duration': 1., 'qpos': reach.tolist(), 'pins': list(FEET),
                             'grasp': {'object_id': 'box'}}]}
    project, warnings = import_motioncreator_project(robot, source)
    assert project['format'] == FORMAT and len(project['keyframes']) == 2
    assert project['source']['project_id'] == 'a' * 32
    [box] = project['scene_objects']
    assert box['kind'] == 'box' and box['name'] == '박스 1' and box['size'] == [.3, .5, .24]
    np.testing.assert_allclose(box['wxyz'], [.70710678, 0, 0, .70710678])   # xyzw -> wxyz
    assert box['color'] == '#ff8400' and box['friction'] == .7
    assert any('GLB' in w for w in warnings) and any('파지' in w for w in warnings)
    clip = dict(source, keyframes=[dict(source['keyframes'][0], samples=[robot.home.tolist()])])
    with pytest.raises(ValueError, match='클립'):
        import_motioncreator_project(robot, clip)


def test_save_bundle_roundtrip(robot, reach, tmp_path):
    project = two_keyframe_project(robot, reach)
    folder, saved = save_bundle(robot, project, fps=30, directory=tmp_path)
    assert {p.name for p in folder.iterdir()} == {'project.json', 'motion.csv', 'metadata.json'}
    qpos = np.loadtxt(folder / 'motion.csv', delimiter=',')
    assert qpos.shape == (31, 36)
    metadata = json.loads((folder / 'metadata.json').read_text())
    assert metadata['samples'] == 31 and metadata['validation']['kind'] == 'kinematic only'
    loaded, warnings = load_project(robot, (folder / 'project.json').read_text())
    assert loaded['project_id'] == project['project_id'] and not warnings

    # Saving again with a new name updates the same project folder instead of creating a copy.
    saved['name'] = 'renamed'
    folder_again, _ = save_bundle(robot, saved, fps=30, directory=tmp_path)
    assert folder_again.name.startswith('renamed_') and not folder.exists()
    assert len(list_saved(tmp_path)) == 1

    copy_folder, copy = save_bundle(robot, saved, fps=30, directory=tmp_path, save_as=True)
    assert copy['project_id'] != saved['project_id'] and len(list_saved(tmp_path)) == 2

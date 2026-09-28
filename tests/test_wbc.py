"""Decoupled WBC 제어기 이식 검증: Motion Creator(MuJoCo) 결과와 같은 모델·같은 기준 궤적에서 비교한다."""
import json
from pathlib import Path

import numpy as np
import pytest

from motionsim.wbc import DecoupledController, authored_waist_to_torso_rpy, ghost_tracking_command, load_parameters

FIXTURES = Path(__file__).parent / 'fixtures'
mujoco = pytest.importorskip('mujoco')


def test_controller_matches_motioncreator_in_mujoco():
    from motionsim.sim_mujoco import MujocoBackend, rollout
    reference = np.load(FIXTURES / 'motioncreator_wbc_reference.npz')
    meta = json.loads((FIXTURES / 'motioncreator_wbc_reference.json').read_text())
    controller = DecoupledController(reference['reference_qpos'])
    recording, fell = rollout(controller, MujocoBackend(), meta['seconds'])
    assert not fell and meta['phase'] != 'fallen'
    ours = np.stack([frame['qpos'] for frame in recording])
    assert ours.shape == reference['qpos'].shape
    # Same model, same policy, same arithmetic: trajectories agree to numerical noise.
    assert np.abs(ours - reference['qpos']).max() < 1e-5
    np.testing.assert_allclose(np.stack([frame['nav'] for frame in recording]), reference['nav'], atol=1e-5)


def test_ghost_command_walks_toward_the_reference():
    p = load_parameters()
    origin = np.array([0, 0, .79, 1, 0, 0, 0.])
    reference = origin.copy()
    following = origin.copy()
    following[0] += .004   # 0.2 m/s forward over one 20 ms control step
    nav, height, tracking = ghost_tracking_command(origin, reference, following, origin, origin, .02, p)
    assert nav[0] == pytest.approx(.2, abs=1e-6) and abs(nav[1]) < 1e-9 and abs(nav[2]) < 1e-9
    assert height == pytest.approx(p['initial_height'])
    assert tracking['root_error_m'] == pytest.approx(0.)


def test_waist_command_is_clipped_to_policy_limits():
    limits = np.asarray(load_parameters()['command_limits']['torso_rpy'])
    rpy = authored_waist_to_torso_rpy([0., 1.0, 0.], limits)
    assert rpy[0] == pytest.approx(limits[0])

"""Isaac vs MuJoCo sim2sim regression (about 1 minute per model, needs the GPU and Isaac Sim).

Run explicitly:  python -m pytest -m isaac
Thresholds come from the first measured run (2026-09-28): Isaac and MuJoCo both finish the 8 s
stand-reach-walk motion; final root differs by ~2 cm and yaw by ~0.1 deg.
"""
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.isaac


def run_script(*args):
    env = dict(os.environ, OMNI_KIT_ACCEPT_EULA='YES', PYTHONPATH=str(ROOT))
    output = subprocess.run([sys.executable, *args], cwd=ROOT, env=env, capture_output=True, text=True, timeout=900).stdout
    line = next(line for line in output.splitlines() if line.startswith('RESULT_JSON '))
    return json.loads(line.removeprefix('RESULT_JSON '))


@pytest.mark.parametrize('model', ['g1', 'g1-tools'])
def test_isaac_matches_mujoco(model):
    result = run_script('scripts/sim2sim_check.py', '--model', model)
    isaac, mujoco = result['isaac'], result['mujoco']
    assert not isaac['fell'] and not mujoco['fell']
    assert abs(isaac['final_root_xy'][0] - mujoco['final_root_xy'][0]) < .08
    assert abs(isaac['final_root_xy'][1] - mujoco['final_root_xy'][1]) < .08
    assert abs(isaac['final_yaw_deg'] - mujoco['final_yaw_deg']) < 3.
    assert isaac['upper_rmse_deg'] < 6.
    assert result['difference']['root_xy_max_m'] < .1


def test_isaac_scene_assets_and_primitives():
    """Measured 2026-09-28: asset meshes match the catalog, assets rest within ~2 mm, boxes land on surfaces."""
    result = run_script('scripts/scene_check.py')
    assert max(result['mesh_size_error_m'].values()) < .002, result['mesh_size_error_m']
    assert max(result['rest_drift_m'].values()) < .005, result['rest_drift_m']
    assert abs(result['box_on_pallet_error_m']) < .005
    assert abs(result['dropped_box_error_m']) < .003

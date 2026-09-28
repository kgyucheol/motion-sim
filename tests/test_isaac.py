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


@pytest.mark.parametrize('model', ['g1', 'g1-tools'])
def test_isaac_matches_mujoco(model):
    env = dict(os.environ, OMNI_KIT_ACCEPT_EULA='YES', PYTHONPATH=str(ROOT))
    output = subprocess.run([sys.executable, 'scripts/sim2sim_check.py', '--model', model], cwd=ROOT, env=env,
                            capture_output=True, text=True, timeout=900).stdout
    line = next(line for line in output.splitlines() if line.startswith('RESULT_JSON '))
    result = json.loads(line.removeprefix('RESULT_JSON '))
    isaac, mujoco = result['isaac'], result['mujoco']
    assert not isaac['fell'] and not mujoco['fell']
    assert abs(isaac['final_root_xy'][0] - mujoco['final_root_xy'][0]) < .08
    assert abs(isaac['final_root_xy'][1] - mujoco['final_root_xy'][1]) < .08
    assert abs(isaac['final_yaw_deg'] - mujoco['final_yaw_deg']) < 3.
    assert isaac['upper_rmse_deg'] < 6.
    assert result['difference']['root_xy_max_m'] < .1

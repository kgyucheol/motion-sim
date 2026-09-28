"""Isaac vs MuJoCo sim2sim check for the Decoupled WBC controller.

Runs the Motion Creator WBC fixture motion (stand, reach, walk 0.3 m with 20 deg yaw) with the same
controller in Isaac Sim and in MuJoCo (policy model), then compares the outcomes.

  OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=. python scripts/sim2sim_check.py [--model g1]
"""
import argparse
import builtins
import functools
import json
import sys
from pathlib import Path

import numpy as np

print = functools.partial(builtins.print, flush=True)   # Isaac exits without flushing stdout
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def summary(recording, fell, reference):
    from scipy.spatial.transform import Rotation
    from motionsim.wbc import tracking_summary
    final = recording[-1]['qpos']
    return {'fell': fell, 'seconds': round(recording[-1]['time'], 2),
            'final_root_xy': np.round(final[:2], 3).tolist(), 'final_root_z': round(float(final[2]), 3),
            'final_yaw_deg': round(float(np.rad2deg(Rotation.from_quat(final[3:7][[1, 2, 3, 0]]).as_euler('xyz')[2])), 1),
            **{k: round(v, 3) for k, v in tracking_summary(recording, reference).items()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='g1', choices=['g1', 'g1-tools'])
    args = parser.parse_args()
    fixture = np.load(ROOT / 'tests/fixtures/motioncreator_wbc_reference.npz')
    seconds = json.loads((ROOT / 'tests/fixtures/motioncreator_wbc_reference.json').read_text())['seconds']
    reference = fixture['reference_qpos']

    from motionsim.sim_mujoco import MujocoBackend, rollout as mujoco_rollout
    from motionsim.wbc import DecoupledController
    mujoco_recording, mujoco_fell = mujoco_rollout(DecoupledController(reference), MujocoBackend(), seconds)

    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})
    try:
        from motionsim.robot import Robot
        from motionsim.sim_isaac import IsaacBackend, rollout as isaac_rollout
        backend = IsaacBackend(Robot(args.model))
        print('ISAAC settings', json.dumps(backend.settings()))
        isaac_recording, isaac_fell = isaac_rollout(DecoupledController(reference), backend, seconds)
        result = {'model': args.model, 'mujoco': summary(mujoco_recording, mujoco_fell, reference),
                  'isaac': summary(isaac_recording, isaac_fell, reference)}
        n = min(len(mujoco_recording), len(isaac_recording))
        diff = np.stack([isaac_recording[i]['qpos'] - mujoco_recording[i]['qpos'] for i in range(n)])
        result['difference'] = {'root_xy_max_m': round(float(np.abs(diff[:, :2]).max()), 3),
                                'joint_rmse_deg': round(float(np.rad2deg(np.sqrt(np.mean(diff[:, 7:] ** 2)))), 2)}
        print('RESULT_JSON', json.dumps(result))
    finally:
        app.close()


if __name__ == '__main__':
    main()

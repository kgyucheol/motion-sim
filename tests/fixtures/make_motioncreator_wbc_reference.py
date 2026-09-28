"""Motion Creator의 Decoupled WBC(MuJoCo, auto 모드)를 실행해 제어기 이식 검증용 기준값을 만든다.

실행 (Motion Creator 정책 환경: mujoco, onnxruntime, Decoupled WBC 자산 필요. 저장소 루트에서):
  MOTIONCREATOR=/home/kim/motioncreator /home/kim/motioncreator/.conda-policy/bin/python \
      tests/fixtures/make_motioncreator_wbc_reference.py

모션: 서기 → 오른손 뻗기(1 s) → 루트를 앞으로 0.3 m, yaw 20° 이동(2 s, 발 고정 없음 → walk 정책).
기준 궤적(reference_qpos, 50 Hz)을 함께 저장하므로 motion-sim 쪽은 보간 구현과 무관하게 제어기만 비교한다.
"""
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
MOTIONCREATOR = Path(os.environ.get('MOTIONCREATOR', '/home/kim/motioncreator'))
sys.path.insert(0, str(MOTIONCREATOR))
os.environ.pop('MOTIONCREATOR_MODEL', None)
from motioncreator.robot import FEET, Robot  # noqa: E402
from motioncreator.motion import new_project  # noqa: E402
from motioncreator.decoupled_wbc import DecoupledSimulation  # noqa: E402

SECONDS = 8.0   # 2 s settle + 5 s motion + 1 s hold


def main():
    robot = Robot('g1')
    home = robot.home
    hand = robot.point(robot.data(home), 'right_hand')[0]
    reach, info = robot.solve(home, home, selected_targets={'right_hand': (hand + [.10, -.05, .08]).tolist()})
    assert info['converged']
    moved = reach.copy()
    moved[0] += .3
    moved[3:7] = Rotation.from_euler('z', 20, degrees=True).as_quat()[[3, 0, 1, 2]]
    project = new_project(robot, 'wbc fixture')
    project['keyframes'] += [
        {'name': 'reach', 'duration': 1.0, 'qpos': reach.tolist(), 'pins': list(FEET), 'angle_pins': []},
        {'name': 'walk', 'duration': 2.0, 'qpos': moved.tolist(), 'pins': [], 'angle_pins': []}]
    project['keyframes'][1]['pins'] = []   # the reach frame starts the unpinned walking segment
    simulation = DecoupledSimulation(robot, project, autostart=False)
    simulation.play()
    steps = int(round(SECONDS / simulation.parameters['simulation_dt']))
    for _ in range(steps):
        simulation.step()
        if simulation.phase == 'fallen':
            break
    frames = simulation.recording_copy()
    revision = subprocess.run(['git', '-C', str(MOTIONCREATOR), 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True).stdout.strip()
    np.savez_compressed(HERE / 'motioncreator_wbc_reference.npz',
                        reference_qpos=simulation.reference['qpos'],
                        time=np.array([f['time'] for f in frames]),
                        qpos=np.stack([f['qpos'] for f in frames]),
                        nav=np.stack([f['nav'] for f in frames]))
    meta = {'source': 'motioncreator decoupled_wbc.DecoupledSimulation, auto mode, g1_gear_wbc.xml',
            'motioncreator_revision': revision, 'seconds': SECONDS, 'steps': simulation.step_count,
            'phase': simulation.phase, 'frames': len(frames)}
    (HERE / 'motioncreator_wbc_reference.json').write_text(json.dumps(meta, indent=1) + '\n')
    final = frames[-1]['qpos']
    print(meta, 'final root', np.round(final[:3], 3), 'yaw deg',
          round(float(np.rad2deg(Rotation.from_quat(final[3:7][[1, 2, 3, 0]]).as_euler('xyz')[2])), 1))


if __name__ == '__main__':
    main()

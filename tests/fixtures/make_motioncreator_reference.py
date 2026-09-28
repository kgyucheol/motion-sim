"""Motion Creator(MuJoCo)의 IK·보간 알고리즘으로 기준값을 만든다. motion-sim 회귀 테스트용 fixture.

- g1: Motion Creator assets/g1/g1.xml은 구형 G1이라 허리 pitch 축이 unitree_ros rev_1_0보다 10 mm 높다.
  알고리즘만 비교하기 위해 Motion Creator 코드를 motion-sim과 같은 URDF로 만든 MuJoCo 모델 위에서 실행한다.
- g1-tools: Motion Creator가 실제로 쓰는 모델(tool_model.py, 주걱=왼손)을 그대로 쓴다. motion-sim URDF도
  주걱=왼손으로 생성했으므로 모델까지 같아야 한다.
모델 차이는 docs/plan.md의 "Motion Creator 모델과의 차이"에 기록한다.

실행 (Motion Creator 코드와 MuJoCo 필요. 저장소 루트에서):
  MOTIONCREATOR=/home/kim/motioncreator /home/kim/motioncreator/.conda/bin/python \
      tests/fixtures/make_motioncreator_reference.py
SciPy 1.15.3과 1.17.1에서 결과가 1e-13 이내로 같음을 확인했다(2026-09-28).
"""
import json
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
MOTIONCREATOR = Path(os.environ.get('MOTIONCREATOR', '/home/kim/motioncreator'))
sys.path.insert(0, str(MOTIONCREATOR))
import motioncreator.robot as mc_robot  # noqa: E402
from motioncreator import motion as mc_motion  # noqa: E402

MODELS = json.loads((REPO / 'integrations/robot-models.json').read_text())['models']


def mjcf_from_urdf(urdf_path):
    """Compile a URDF like Motion Creator's tool_model_xml, without mirroring, and add a free root joint."""
    urdf_path = Path(urdf_path)
    source = ET.parse(urdf_path).getroot()
    compiler = source.find('mujoco/compiler')
    compiler.set('meshdir', str(urdf_path.parent / 'meshes'))
    compiler.set('strippath', 'true')
    compiler.set('fusestatic', 'false')
    model = mujoco.MjModel.from_xml_string(ET.tostring(source, encoding='unicode'))
    with tempfile.NamedTemporaryFile(suffix='.xml') as output:
        mujoco.mj_saveLastXML(output.name, model)
        root = ET.parse(output.name).getroot()
    root.find('compiler').set('meshdir', str(urdf_path.parent / 'meshes'))
    pelvis = root.find("worldbody/body[@name='pelvis']")
    pelvis.set('pos', '0 0 .79')
    pelvis.insert(0, ET.Element('freejoint', name='floating_base'))
    return root


def motioncreator_robot(model_id, workdir):
    """g1: Motion Creator Robot on the motion-sim base URDF. g1-tools: Motion Creator's own tool model."""
    if model_id == 'g1-tools':
        return mc_robot.Robot('g1-tools')
    path = Path(workdir) / f'{model_id}.xml'
    ET.ElementTree(mjcf_from_urdf(REPO / MODELS[model_id]['urdf'])).write(path)
    mc_robot.MODEL_PATH = path
    return mc_robot.Robot('g1')


def main():
    revision = subprocess.run(['git', '-C', str(MOTIONCREATOR), 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True).stdout.strip()
    rng = np.random.default_rng(20260928)
    arrays = {}
    meta = {'source': 'motioncreator robot.py/motion.py on motion-sim URDFs', 'motioncreator_revision': revision,
            'models': {}}
    names = sorted(mc_robot.HANDLES)
    with tempfile.TemporaryDirectory() as workdir:
        for model_id in ('g1', 'g1-tools'):
            robot = motioncreator_robot(model_id, workdir)
            key = model_id.replace('-', '_')
            meta['models'][model_id] = {'joint_names': robot.names, 'handles': names}
            qs = [robot.home.copy()]
            for _ in range(24):
                q = robot.home.copy()
                lower, upper = robot.lower[3:], robot.upper[3:]
                q[7:] = lower + (upper - lower) * rng.uniform(.15, .85, size=lower.size)
                q[:3] = rng.uniform([-.5, -.5, .5], [.5, .5, 1.])
                quat = rng.normal(size=4)
                q[3:7] = quat / np.linalg.norm(quat)
                qs.append(q)
            qs = np.stack(qs)
            positions = np.zeros((len(qs), len(names), 3))
            rotations = np.zeros((len(qs), len(names), 3, 3))
            for i, q in enumerate(qs):
                d = robot.data(q)
                for j, name in enumerate(names):
                    positions[i, j], rotations[i, j] = robot.point(d, name)
            arrays.update({f'{key}_fk_q': qs, f'{key}_fk_pos': positions, f'{key}_fk_rot': rotations,
                           f'{key}_home': robot.home})

            home = robot.home
            d = robot.data(home)
            right_hand = robot.point(d, 'right_hand')[0]
            scenarios = [
                ('right_hand_forward', {'selected_targets': {'right_hand': (right_hand + [.10, 0, .05]).tolist()}}),
                ('left_hand_orientation', {'orientation_targets': {'left_hand': [0, .258819, 0, .9659258]}}),
                ('left_elbow_joint', {'joint_targets': {'left_elbow_joint': .9}}),
                ('pelvis_down', {'selected_targets': {'pelvis': (robot.point(d, 'pelvis')[0] + [0, 0, -.08]).tolist()}}),
                ('right_hand_free_mode', {'selected_targets': {'right_hand': (right_hand + [.05, -.05, .10]).tolist()},
                                          'mode': 'free'}),
            ]
            results = []
            for name, kwargs in scenarios:
                q, info = robot.solve(home, home, **kwargs)
                results.append({'name': name, 'kwargs': kwargs, 'q': q.tolist(),
                                'info': {k: v for k, v in info.items() if isinstance(v, (bool, int, float))}})
            meta['models'][model_id]['ik'] = results

            reach, _ = robot.solve(home, home, **scenarios[0][1])
            down, _ = robot.solve(home, home, **scenarios[3][1])
            project = mc_motion.new_project(robot, 'fixture')
            project['keyframes'] += [
                {'name': 'reach', 'duration': 1.0, 'qpos': reach.tolist(), 'pins': list(mc_robot.FEET), 'angle_pins': []},
                {'name': 'down', 'duration': 1.5, 'qpos': down.tolist(), 'pins': list(mc_robot.FEET), 'angle_pins': []}]
            motion = mc_motion.compile_motion(robot, project, 30)
            arrays[f'{key}_motion_qpos'], arrays[f'{key}_motion_time'] = motion['qpos'], motion['time']
            meta['models'][model_id]['motion_keyframes'] = [
                {k: frame[k] for k in ('name', 'duration', 'qpos', 'pins', 'angle_pins')} for frame in project['keyframes']]
            meta['models'][model_id]['motion_max_pin_error_mm'] = motion['max_pin_error_mm']
            print(model_id, 'IK', [(r['name'], round(r['info']['target_error_mm'], 2), r['info']['rejected'])
                                   for r in results], 'motion samples', len(motion['time']))
    np.savez_compressed(HERE / 'motioncreator_reference.npz', **arrays)
    (HERE / 'motioncreator_reference.json').write_text(json.dumps(meta, indent=1, ensure_ascii=False) + '\n')


if __name__ == '__main__':
    main()

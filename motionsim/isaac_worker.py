"""Persistent Isaac Sim worker: runs authored motions with the Decoupled WBC controller.

Isaac Sim takes ~40 s to start, so the editor starts this process once and reuses it. Each job
resets the same stage. Protocol (multiprocessing.connection, localhost, authkey):
  editor -> worker  {'type': 'run', 'project': dict, 'hold_seconds': float}
                    {'type': 'asset_mesh', 'asset': catalog path}
  worker -> editor  {'type': 'progress', 'value': 0..1}
                    {'type': 'result', 'frames': [...], 'summary': {...}, 'settings': {...}}
                    {'type': 'asset_mesh', 'asset': str, 'vertices': (N, 3) m, 'faces': (M, 3)}
                    {'type': 'error', 'message': str}
  on connect        {'type': 'ready', 'model': str}

    OMNI_KIT_ACCEPT_EULA=YES python -m motionsim.isaac_worker --model g1 --port 8091
"""
import argparse
import builtins
import functools
import os
import sys
import traceback
from multiprocessing.connection import Listener
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REFERENCE_FPS = 50   # Decoupled WBC consumes the authored motion at its 50 Hz control rate
print = functools.partial(builtins.print, flush=True)


def authkey():
    return os.environ.get('MOTIONSIM_WORKER_KEY', 'motionsim-local').encode()


def run_job(robot, backend, project, hold_seconds, send):
    import numpy as np
    from scipy.spatial.transform import Rotation
    from .motion import compile_motion
    from .sim_isaac import rollout
    from .wbc import DecoupledController, tracking_summary, verify_assets
    verify_assets()
    reference = compile_motion(robot, project, fps=REFERENCE_FPS)['qpos']
    controller = DecoupledController(reference)
    backend.set_scene(project.get('scene_objects', []))
    seconds = controller.p['upper_body_settle_seconds'] + controller.duration + hold_seconds
    recording, fell = rollout(controller, backend, seconds, progress=lambda value: send({'type': 'progress', 'value': value}))
    final = recording[-1]['qpos'] if recording else reference[0]
    summary = {'fell': fell, 'phase': controller.phase, 'seconds': float(recording[-1]['time']) if recording else 0.,
               'settle_seconds': controller.p['upper_body_settle_seconds'], 'motion_seconds': controller.duration,
               'final_yaw_deg': float(np.rad2deg(Rotation.from_quat(final[3:7][[1, 2, 3, 0]]).as_euler('xyz')[2])),
               **tracking_summary(recording, reference), 'objects': object_summary(project, recording)}
    frames = [{'time': float(f['time']), 'motion_time': float(f['motion_time']), 'qpos': f['qpos'].tolist(),
               'objects': f.get('objects', {})} for f in recording]
    return {'type': 'result', 'frames': frames, 'summary': summary, 'settings': backend.settings()}


def object_summary(project, recording):
    """How far each object moved from its authored pose by the end (m, deg)."""
    import numpy as np
    from scipy.spatial.transform import Rotation
    if not recording or 'objects' not in recording[-1]:
        return {}
    summary = {}
    for item in project.get('scene_objects', []):
        position, wxyz = recording[-1]['objects'][item['id']]
        start = Rotation.from_quat(np.asarray(item['wxyz'])[[1, 2, 3, 0]])
        end = Rotation.from_quat(np.asarray(wxyz)[[1, 2, 3, 0]])
        summary[item['id']] = {'name': item['name'],
                               'moved_m': float(np.linalg.norm(np.asarray(position) - np.asarray(item['position']))),
                               'rotated_deg': float(np.rad2deg((end * start.inv()).magnitude()))}
    return summary


def exit_with_parent(parent_pid):
    """Isaac holds GPU memory on a shared server: never outlive the editor that started us."""
    import threading
    import time

    def watch():
        while True:
            try:
                os.kill(parent_pid, 0)
            except OSError:
                print(f'WORKER parent {parent_pid} is gone; exiting')
                os._exit(0)
            time.sleep(2.)
    threading.Thread(target=watch, daemon=True).start()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', default='g1', choices=['g1', 'g1-tools'])
    parser.add_argument('--port', type=int, default=8091)
    parser.add_argument('--parent-pid', type=int, help='exit when this process ends (set by the editor)')
    args = parser.parse_args()
    if args.parent_pid:
        exit_with_parent(args.parent_pid)
    sys.path.insert(0, str(ROOT))
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})
    try:
        from motionsim.robot import Robot
        from motionsim.sim_isaac import IsaacBackend
        robot = Robot(args.model)
        backend = IsaacBackend(robot)
        with Listener(('127.0.0.1', args.port), authkey=authkey()) as listener:
            print(f'WORKER ready model={args.model} port={args.port}')
            while True:
                with listener.accept() as connection:
                    connection.send({'type': 'ready', 'model': args.model})
                    while True:
                        try:
                            request = connection.recv()
                        except EOFError:
                            break
                        if request.get('type') == 'shutdown':
                            return
                        try:
                            if request.get('type') == 'run':
                                connection.send(run_job(robot, backend, request['project'],
                                                        float(request.get('hold_seconds', 1.)), connection.send))
                            elif request.get('type') == 'asset_mesh':
                                from motionsim.sim_isaac import export_asset_mesh
                                vertices, faces = export_asset_mesh(request['asset'])
                                connection.send({'type': 'asset_mesh', 'asset': request['asset'],
                                                 'vertices': vertices, 'faces': faces})
                            else:
                                connection.send({'type': 'error', 'message': f'unknown request {request.get("type")}'})
                        except Exception as error:   # report any failure to the editor instead of dying
                            traceback.print_exc()
                            connection.send({'type': 'error', 'message': f'{type(error).__name__}: {error}'})
    finally:
        app.close()


if __name__ == '__main__':
    main()

"""Isaac scene physics check for the catalog assets and primitives (needs network access to the Isaac assets).

  OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=. python scripts/scene_check.py

Checks: exported asset meshes match the catalog size; every asset rests without drifting; a box dropped
onto the fixed pallet stops on its top surface; a box dropped from 0.5 m rests at half its height.
"""
import builtins
import functools
import json
import sys
from pathlib import Path

import numpy as np

print = functools.partial(builtins.print, flush=True)   # Isaac exits without flushing stdout
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})
    try:
        from motionsim.robot import Robot
        from motionsim.scene import CATALOG, new_object, rest_on_ground, world_vertices
        from motionsim.sim_isaac import IsaacBackend, export_asset_mesh
        result = {'mesh_size_error_m': {}, 'rest_drift_m': {}}
        meshes = {}
        for asset in CATALOG['assets']:
            vertices, _ = export_asset_mesh(asset['path'])
            meshes[asset['path']] = vertices
            result['mesh_size_error_m'][asset['name']] = float(np.abs(vertices.max(0) - vertices.min(0) - asset['size_m']).max())
        objects = []
        for index, asset in enumerate(CATALOG['assets']):
            item = new_object('asset', objects, asset['path'])
            item['position'] = [-4 + 1.6 * index, 3., 0.]
            rest_on_ground(item, meshes[asset['path']])
            objects.append(item)
        pallet = next(item for item in objects if 'Pallet' in item['asset'])
        on_pallet = new_object('box', objects)
        on_pallet.update(size=[.2, .2, .2], position=[pallet['position'][0], pallet['position'][1], .6])
        dropped = new_object('box', objects)
        dropped.update(size=[.3, .3, .3], position=[0., -3., .5])
        objects += [on_pallet, dropped]
        backend = IsaacBackend(Robot('g1'))
        backend.set_scene(objects)
        backend.reset(np.zeros(29))
        start = backend.object_poses()
        for _ in range(400):   # 2 s
            backend.world.step(render=False)
        end = backend.object_poses()
        for item in objects[:len(CATALOG['assets'])]:
            result['rest_drift_m'][item['name']] = float(np.linalg.norm(np.subtract(end[item['id']][0], start[item['id']][0])))
        pallet_top = float(world_vertices(pallet, meshes[pallet['asset']])[:, 2].max())
        result['box_on_pallet_error_m'] = float(end[on_pallet['id']][0][2] - (pallet_top + .1))
        result['dropped_box_error_m'] = float(end[dropped['id']][0][2] - .15)
        print('RESULT_JSON', json.dumps(result))
    finally:
        app.close()


if __name__ == '__main__':
    main()

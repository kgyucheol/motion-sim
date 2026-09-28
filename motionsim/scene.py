"""Scene objects: primitives and Isaac assets placed next to the robot.

Object contract (project `scene_objects`, one dict per object):
  id        'obj_<8 hex>' (stable across edits)
  name      display name
  kind      'box' | 'sphere' | 'cylinder' | 'asset'
  asset     Isaac asset path relative to the asset root (kind 'asset' only)
  position  [x, y, z] m, world frame; primitives: geometric centre, assets: asset root origin
  wxyz      orientation quaternion (motion-sim convention; Motion Creator used xyzw)
  size      primitives only, full extents m: box [x, y, z], sphere [d, d, d], cylinder [d, d, h]
  scale     assets only, uniform scale on top of the asset's own units
  mass_kg   primitives: required; assets: null keeps the asset's authored mass
  friction  static = dynamic friction; primitives only (assets keep their authored material)
  color     '#rrggbb', opacity 0..1 (display only)
  fixed     True: static collider that never moves
"""
from pathlib import Path
import hashlib
import json
import os
import re
import uuid

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
CATALOG = json.loads((ROOT / 'integrations/isaac-assets.json').read_text())
CATALOG_PATHS = {item['path']: item for item in CATALOG['assets']}
PRIMITIVES = ('box', 'sphere', 'cylinder')
KINDS = (*PRIMITIVES, 'asset')
MAX_OBJECTS = 32
ID_PATTERN = re.compile(r'obj_[0-9a-f]{8}')
COLOR_PATTERN = re.compile(r'#[0-9a-fA-F]{6}')
DEFAULTS = {'box': ([.3, .3, .3], '상자', '#c8a26b'), 'sphere': ([.2, .2, .2], '공', '#5b8def'),
            'cylinder': ([.15, .15, .3], '원기둥', '#7bb661')}
ASSET_CACHE = Path(os.environ.get('MOTIONSIM_ASSET_CACHE', Path.home() / '.cache/motionsim/isaac_assets'))


def new_object(kind, existing, asset=None):
    """A new object in front of the robot, resting on the ground, with a unique id and name."""
    if kind not in KINDS:
        raise ValueError(f'알 수 없는 물체 종류입니다: {kind}')
    ids = {item['id'] for item in existing}
    identifier = f'obj_{uuid.uuid4().hex[:8]}'
    while identifier in ids:
        identifier = f'obj_{uuid.uuid4().hex[:8]}'
    if kind == 'asset':
        if asset not in CATALOG_PATHS:
            raise ValueError(f'카탈로그에 없는 Isaac 에셋입니다: {asset}')
        base = CATALOG_PATHS[asset]['name']
        item = {'kind': 'asset', 'asset': asset, 'scale': 1., 'mass_kg': None, 'friction': None, 'color': '#b0b0b0'}
    else:
        size, base, color = DEFAULTS[kind]
        item = {'kind': kind, 'size': list(size), 'mass_kg': 1., 'friction': .8, 'color': color}
    names = {entry['name'] for entry in existing}
    number = 1
    while f'{base} {number}' in names:
        number += 1
    item.update({'id': identifier, 'name': f'{base} {number}', 'position': [.6, 0., 0.], 'wxyz': [1., 0., 0., 0.],
                 'opacity': 1., 'fixed': fixed_only(item)})
    return item


def fixed_only(item):
    """Assets whose movable part is only an accessory (e.g. the packing table) are always static."""
    return item['kind'] == 'asset' and bool(CATALOG_PATHS.get(item.get('asset'), {}).get('fixed_only'))


def _vector(item, field, length):
    value = np.asarray(item.get(field), dtype=float)
    if value.shape != (length,) or not np.isfinite(value).all():
        raise ValueError(f'물체 {field}에는 유한한 숫자 {length}개가 필요합니다')
    return value


def validate_objects(objects):
    """Validate and normalise a scene object list in place."""
    if not isinstance(objects, list) or len(objects) > MAX_OBJECTS:
        raise ValueError(f'장면 물체는 최대 {MAX_OBJECTS}개입니다')
    seen = set()
    for item in objects:
        if not isinstance(item, dict):
            raise ValueError('장면 물체는 객체여야 합니다')
        if not ID_PATTERN.fullmatch(str(item.get('id'))) or item['id'] in seen:
            raise ValueError('물체 id는 obj_ 뒤 hex 8자이고 서로 달라야 합니다')
        seen.add(item['id'])
        if not isinstance(item.get('name'), str) or not 1 <= len(item['name']) <= 80:
            raise ValueError('물체 이름은 1–80자여야 합니다')
        kind = item.get('kind')
        if kind not in KINDS:
            raise ValueError(f'물체 종류는 {", ".join(KINDS)} 중 하나여야 합니다')
        position = _vector(item, 'position', 3)
        if np.abs(position).max() > 10:
            raise ValueError('물체 위치는 원점에서 10 m 이내여야 합니다')
        wxyz = _vector(item, 'wxyz', 4)
        if abs(np.linalg.norm(wxyz) - 1) > 1e-4:
            raise ValueError('물체 방향 quaternion(wxyz)이 정규화되어 있지 않습니다')
        item['wxyz'] = (wxyz / np.linalg.norm(wxyz)).tolist()
        if kind == 'asset':
            if item.get('asset') not in CATALOG_PATHS:
                raise ValueError(f'카탈로그에 없는 Isaac 에셋입니다: {item.get("asset")}')
            if not .05 <= float(item.get('scale', 1.)) <= 20:
                raise ValueError('에셋 배율은 0.05–20이어야 합니다')
            if item.get('mass_kg') is not None and not .001 <= float(item['mass_kg']) <= 500:
                raise ValueError('물체 질량은 0.001–500 kg이어야 합니다')
        else:
            size = _vector(item, 'size', 3)
            if size.min() < .01 or size.max() > 5:
                raise ValueError('물체 크기는 0.01–5 m여야 합니다')
            if kind == 'sphere' and not np.allclose(size, size[0]):
                raise ValueError('공은 세 방향 크기가 같아야 합니다')
            if kind == 'cylinder' and abs(size[0] - size[1]) > 1e-9:
                raise ValueError('원기둥은 x, y 지름이 같아야 합니다')
            if not .001 <= float(item.get('mass_kg', 0)) <= 500:
                raise ValueError('물체 질량은 0.001–500 kg이어야 합니다')
            if not 0 <= float(item.get('friction', -1)) <= 2:
                raise ValueError('마찰 계수는 0–2여야 합니다')
        if not COLOR_PATTERN.fullmatch(str(item.get('color', ''))):
            raise ValueError('물체 색은 #rrggbb 형식이어야 합니다')
        if not 0 <= float(item.get('opacity', 1.)) <= 1:
            raise ValueError('물체 투명도는 0–1이어야 합니다')
        if not isinstance(item.get('fixed', False), bool):
            raise ValueError('물체 fixed는 true/false여야 합니다')
        if fixed_only(item) and not item.get('fixed'):
            raise ValueError(f'{item["name"]}은(는) 고정 물체로만 쓸 수 있는 에셋입니다')
    return objects


def from_motioncreator(item, existing):
    """Convert one Motion Creator scene object. Returns (object or None, warning or None)."""
    shape = item.get('shape')
    name = str(item.get('name') or shape)[:80]
    if item.get('asset_id'):
        return None, f'{name}: 가져온 GLB 물체는 지원하지 않아 건너뛰었습니다'
    if shape not in PRIMITIVES:
        return None, f'{name}: {shape} 형태는 지원하지 않아 건너뛰었습니다'
    converted = new_object(shape, existing)
    xyzw = np.asarray(item.get('quaternion_xyzw', [0, 0, 0, 1]), dtype=float)
    converted.update({'name': name, 'position': [float(v) for v in item['position']], 'wxyz': xyzw[[3, 0, 1, 2]].tolist(),
                      'size': [float(v) for v in item['size']], 'mass_kg': float(item.get('mass_kg', 1.)),
                      'friction': float(item.get('friction', .8)), 'fixed': bool(item.get('fixed', False)),
                      'opacity': float(item.get('opacity', 1.))})
    if COLOR_PATTERN.fullmatch(str(item.get('color', ''))):
        converted['color'] = item['color']
    return converted, None


def primitive_mesh(item):
    """Display/geometry mesh of a primitive in its own frame (centre at the origin)."""
    import trimesh
    size = np.asarray(item['size'], dtype=float)
    if item['kind'] == 'box':
        mesh = trimesh.creation.box(extents=size)
    elif item['kind'] == 'sphere':
        mesh = trimesh.creation.icosphere(subdivisions=3, radius=size[0] / 2)
    else:
        mesh = trimesh.creation.cylinder(radius=size[0] / 2, height=size[2], sections=48)
    return np.asarray(mesh.vertices, dtype=np.float32), np.asarray(mesh.faces, dtype=np.int32)


def asset_cache_path(asset):
    return ASSET_CACHE / f'{hashlib.sha1(asset.encode()).hexdigest()[:16]}.npz'


def cached_asset_mesh(asset):
    """(vertices in metres, faces) from the local cache, or None when the worker has not exported it yet."""
    path = asset_cache_path(asset)
    if not path.is_file():
        return None
    data = np.load(path)
    return data['vertices'], data['faces']


def store_asset_mesh(asset, vertices, faces, max_faces=20000):
    """Cache an exported asset mesh, decimated for the browser."""
    vertices, faces = np.asarray(vertices, dtype=np.float32), np.asarray(faces, dtype=np.int32)
    if len(faces) > max_faces:
        import fast_simplification
        vertices, faces = fast_simplification.simplify(vertices, faces, target_reduction=1 - max_faces / len(faces))
    ASSET_CACHE.mkdir(parents=True, exist_ok=True)
    path = asset_cache_path(asset)
    temporary = path.with_name(f'.{path.name}.{uuid.uuid4().hex}.tmp.npz')
    np.savez_compressed(temporary, vertices=vertices, faces=faces)
    os.replace(temporary, path)
    return vertices, faces


def object_mesh(item):
    """Mesh in the object frame, scaled; None for an asset that is not cached yet."""
    if item['kind'] != 'asset':
        return primitive_mesh(item)
    cached = cached_asset_mesh(item['asset'])
    if cached is None:
        return None
    return cached[0] * float(item.get('scale', 1.)), cached[1]


def world_vertices(item, vertices):
    rotation = Rotation.from_quat(np.asarray(item['wxyz'])[[1, 2, 3, 0]])
    return rotation.apply(vertices) + np.asarray(item['position'])


def rest_on_ground(item, vertices):
    """Move the object vertically so its lowest point touches z = 0."""
    lowest = world_vertices(item, vertices)[:, 2].min()
    item['position'] = [item['position'][0], item['position'][1], float(item['position'][2] - lowest)]
    return item

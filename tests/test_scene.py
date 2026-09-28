"""장면 물체 계약: 생성, 검증, 바닥 배치, 고정 전용 에셋, 저장 왕복."""
import numpy as np
import pytest

from motionsim.motion import load_project, new_project, save_bundle, validate_project
from motionsim.robot import Robot
from motionsim.scene import (CATALOG, new_object, object_mesh, primitive_mesh, rest_on_ground, validate_objects,
                             world_vertices)


def test_new_objects_get_unique_ids_and_names():
    objects = []
    for _ in range(3):
        objects.append(new_object('box', objects))
    assert len({item['id'] for item in objects}) == 3
    assert [item['name'] for item in objects] == ['상자 1', '상자 2', '상자 3']
    validate_objects(objects)


@pytest.mark.parametrize('kind', ['box', 'sphere', 'cylinder'])
def test_rest_on_ground_touches_z0(kind):
    item = new_object(kind, [])
    item['wxyz'] = [.9238795, .3826834, 0, 0]   # tilted 45 deg about x
    item['position'] = [0, 0, 2.]
    vertices, _ = primitive_mesh(item)
    rest_on_ground(item, vertices)
    assert world_vertices(item, vertices)[:, 2].min() == pytest.approx(0, abs=1e-6)


def test_primitive_mesh_matches_size():
    box = new_object('box', [])
    box['size'] = [.4, .2, .1]
    vertices, _ = primitive_mesh(box)
    np.testing.assert_allclose(vertices.max(0) - vertices.min(0), [.4, .2, .1], atol=1e-6)


@pytest.mark.parametrize('field, value, message', [
    ('size', [.001, .3, .3], '크기'),
    ('wxyz', [2, 0, 0, 0], 'quaternion'),
    ('friction', 3., '마찰'),
    ('color', 'red', '#rrggbb'),
    ('kind', 'cone', '종류'),
])
def test_validation_rejects_bad_objects(field, value, message):
    item = new_object('box', [])
    item[field] = value
    with pytest.raises(ValueError, match=message):
        validate_objects([item])


def test_sphere_and_cylinder_shape_rules():
    sphere = new_object('sphere', [])
    sphere['size'] = [.2, .3, .2]
    with pytest.raises(ValueError, match='공'):
        validate_objects([sphere])
    cylinder = new_object('cylinder', [])
    cylinder['size'] = [.2, .3, .5]
    with pytest.raises(ValueError, match='원기둥'):
        validate_objects([cylinder])


def test_fixed_only_assets_stay_fixed():
    fixed_only = [item['path'] for item in CATALOG['assets'] if item.get('fixed_only')]
    assert fixed_only, 'catalog should mark the pallet and packing table as fixed-only'
    pallet = new_object('asset', [], fixed_only[0])
    assert pallet['fixed'] is True
    pallet['fixed'] = False
    with pytest.raises(ValueError, match='고정 물체로만'):
        validate_objects([pallet])


def test_uncached_asset_has_no_mesh_yet(tmp_path, monkeypatch):
    import motionsim.scene as scene
    monkeypatch.setattr(scene, 'ASSET_CACHE', tmp_path)
    item = new_object('asset', [], CATALOG['assets'][0]['path'])
    assert object_mesh(item) is None
    scene.store_asset_mesh(item['asset'], np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0.]]), np.array([[0, 1, 2]]))
    vertices, faces = object_mesh(item)
    assert faces.shape == (1, 3)


def test_scene_objects_survive_save_and_open(tmp_path):
    robot = Robot('g1')
    project = new_project(robot, 'scene')
    box = new_object('box', project['scene_objects'])
    project['scene_objects'].append(box)
    folder, _ = save_bundle(robot, project, fps=30, directory=tmp_path)
    loaded, _ = load_project(robot, (folder / 'project.json').read_text())
    assert loaded['scene_objects'] == [box]


def test_old_projects_without_scene_objects_are_valid():
    robot = Robot('g1')
    project = new_project(robot)
    del project['scene_objects']
    assert validate_project(robot, project)['scene_objects'] == []

"""편집기 상태 로직(키프레임 편집, 실행 취소, 타임라인 조작, 저장)을 브라우저 없이 검증한다."""
import time

import numpy as np
import pytest
import viser

from motionsim.app import Editor
from motionsim.robot import Robot


@pytest.fixture(scope='module')
def editor(tmp_path_factory):
    server = viser.ViserServer(host='127.0.0.1', port=8097, verbose=False)
    yield Editor(server, Robot('g1'), tmp_path_factory.mktemp('motions'))
    server.stop()


def wait_for_ik(editor, previous_q, timeout=5.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not np.allclose(editor.q, previous_q):
            return
        time.sleep(.02)
    raise AssertionError('IK did not update the pose')


def test_joint_edit_add_keyframe_and_undo(editor):
    editor._load_into_editor(dict(editor.project))  # clears undo history
    before = editor.q.copy()
    editor.joint_sliders['left_elbow_joint'].value = 60.   # programmatic set: ignored by by_user
    editor._on_joint_slider('left_elbow_joint')
    wait_for_ik(editor, before)
    assert editor.q[7 + editor.robot.names.index('left_elbow_joint')] == pytest.approx(np.deg2rad(60), abs=np.deg2rad(1))

    editor._add_keyframe()
    assert len(editor.project['keyframes']) == 2 and editor.keyframe_index == 1
    editor._undo()
    assert len(editor.project['keyframes']) == 1
    editor._redo()
    assert len(editor.project['keyframes']) == 2
    editor._undo()
    editor._undo()  # joint edit
    np.testing.assert_allclose(editor.q, before)


def test_timeline_move_retimes_neighbours(editor):
    editor._add_keyframe()   # keyframe 2 at 1 s
    editor._add_keyframe()   # keyframe 3 at 2 s
    editor._on_timeline_move(editor.timeline_ids[1], 45)
    durations = [frame['duration'] for frame in editor.project['keyframes'][1:]]
    assert durations == pytest.approx([1.5, .5])
    editor._on_timeline_move(editor.timeline_ids[1], 70)   # would pass keyframe 3: rejected
    assert [frame['duration'] for frame in editor.project['keyframes'][1:]] == pytest.approx([1.5, .5])


def test_scene_objects_edit_undo_and_status(editor):
    editor._load_into_editor(dict(editor.project, scene_objects=[]))
    ui = editor.scene_ui
    ui.add('box')
    ui.add('sphere')
    assert [item['kind'] for item in editor.project['scene_objects']] == ['box', 'sphere']
    sphere = editor.project['scene_objects'][1]
    assert ui.selected == sphere['id'] and sphere['id'] in ui.nodes
    assert sphere['position'][2] == pytest.approx(.1)             # resting on the ground (diameter 0.2)
    assert sphere['position'][1] != editor.project['scene_objects'][0]['position'][1]   # placed beside the box
    editor.select('right_hand')                                  # selecting a robot part releases the object
    assert ui.selected is None and ui.gizmo is None
    ui.select(sphere['id'])
    ui.delete()
    assert len(editor.project['scene_objects']) == 1
    editor._undo()
    assert len(editor.project['scene_objects']) == 2 and set(ui.nodes) == {i['id'] for i in editor.project['scene_objects']}
    assert '2개' in editor.gui_status.content


def test_save_and_reopen(editor):
    editor.project['name'] = 'app test'
    editor._save(save_as=False)
    assert editor._message[0] == 'ok'
    count = len(editor.project['keyframes'])
    editor._load_into_editor(dict(editor.project))
    editor.gui_saved.value = next(iter(editor.saved_folders))
    editor._open_saved()
    assert editor.project['name'] == 'app test' and len(editor.project['keyframes']) == count
    assert not editor.undo_stack

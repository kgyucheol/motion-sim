"""viser kinematic motion editor.

Run on the GPU server and open http://<server>:8080 in a local browser:
    python -m motionsim.app --model g1 --port 8080
"""
import argparse
import copy
import html
import signal
import sys
import threading
import time
from pathlib import Path

import fast_simplification
import numpy as np
import trimesh
import viser

from .isaac_client import IsaacWorker
from .motion import (DEFAULT_MOTIONS_DIR, MAX_DURATION, MIN_DURATION, compile_motion, keyframe_times,
                     list_saved, load_project, new_project, save_bundle)
from .robot import ANGLE_LOCKABLE, FEET, HANDLES, ROTATABLE, Robot

FPS = 30
# Clickable body parts, one selection-button row each: row label -> {button label: handle}.
# Individual joints use the joint sliders.
PART_ROWS = {
    '몸통': {'골반': 'pelvis', '허리': 'waist'},
    '손': {'왼쪽': 'left_hand', '오른쪽': 'right_hand'},
    '팔꿈치': {'왼쪽': 'left_elbow', '오른쪽': 'right_elbow'},
    '어깨': {'왼쪽': 'left_shoulder', '오른쪽': 'right_shoulder'},
    '발': {'왼쪽': 'left_foot', '오른쪽': 'right_foot'},
    '무릎': {'왼쪽': 'left_knee', '오른쪽': 'right_knee'},
    '고관절': {'왼쪽': 'left_hip', '오른쪽': 'right_hip'},
}
MARKER_HANDLES = tuple(key for row in PART_ROWS.values() for key in row.values())
JOINT_GROUPS = (('왼다리', ('left_hip', 'left_knee', 'left_ankle')), ('오른다리', ('right_hip', 'right_knee', 'right_ankle')),
                ('허리', ('waist',)), ('왼팔', ('left_shoulder', 'left_elbow', 'left_wrist')),
                ('오른팔', ('right_shoulder', 'right_elbow', 'right_wrist')))
COLORS = {'normal': (60, 130, 255), 'selected': (255, 196, 30), 'pinned': (230, 60, 60)}
MESSAGE_COLORS = {'info': '#1f6feb', 'ok': '#1a7f37', 'warn': '#9a6700', 'error': '#cf222e'}
BRAND = (38, 110, 214)
UNDO_LIMIT = 100
EPSILON = 1e-6
HELP = """
- **선택**: 로봇의 파란 구를 클릭하거나 위 버튼을 누릅니다. 빨간 구는 위치가 고정된 부위입니다.
- **이동·회전**: 기즈모의 화살표를 끌면 이동, 고리를 끌면 회전합니다. 전신 IK가 고정을 지키며 자세를 맞춥니다.
- **키프레임**: `키프레임` 탭에서 현재 자세를 추가합니다. 하단 타임라인의 마커를 끌면 구간 시간이 바뀌고, 눈금을 누르면 그 시점의 보간 자세가 나옵니다.
- **관절**: `관절` 탭 슬라이더로 개별 관절각을 바꿉니다. 고정된 부위는 유지됩니다.
- 편집 결과는 기구학 자세입니다. 균형과 추종은 `검증` 탭에서 Isaac 물리로 확인합니다.
"""
ISAAC_HELP = """
Isaac Sim 물리에서 **GR00T Decoupled WBC**로 현재 키프레임 모션을 추종합니다.
다리와 허리는 정책(Balance/Walk)이, 팔은 저작 궤적 PD가 제어합니다. 루트 궤적은 걷기 속도 명령으로 바뀝니다.
처음 2초는 정책이 자세를 잡는 준비 구간이고, 그 뒤 모션을 재생합니다.
워커는 한 번 띄우면 계속 재사용합니다(첫 시작 약 40초).
"""


def wxyz_to_xyzw(wxyz):
    return np.asarray(wxyz, dtype=float)[[1, 2, 3, 0]]


def quaternion_angle(a, b):
    return 2 * np.arccos(min(1., abs(float(np.dot(a, b)))))


def by_user(handler):
    """Wrap a GUI callback so it ignores programmatic value changes (viser reports them with client=None)."""
    def callback(event):
        if getattr(event, 'client', None) is not None:
            handler()
    return callback


def display_mesh(path, full):
    """Load a visual mesh; decimate large ones for the browser. IK and Isaac always use the URDF meshes."""
    mesh = trimesh.load(path, force='mesh')
    if full or len(mesh.faces) <= 4000:
        return mesh.vertices, mesh.faces
    return fast_simplification.simplify(mesh.vertices.astype(np.float32), mesh.faces.astype(np.int32), target_reduction=.8)


class Editor:
    def __init__(self, server: viser.ViserServer, robot: Robot, motions_dir: Path, full_meshes=False, isaac_port=8091):
        self.server, self.robot, self.motions_dir = server, robot, Path(motions_dir)
        self.lock = threading.RLock()
        self.project = new_project(robot)
        self.q = robot.home.copy()
        self.pins, self.angle_pins = list(FEET), []
        self.keyframe_index = 0
        self.selected = None
        self.anchor = self.q.copy()
        self.drag_start = None
        self.motion = None
        self.playing = False
        self.timeline_ids = []
        self.undo_stack, self.redo_stack = [], []
        self._last_checkpoint = (None, 0.)
        self._ik_request = None
        self._ik_condition = threading.Condition()
        self._message = ('info', '준비되었습니다.')
        self._ik_summary = ''
        self.isaac = IsaacWorker(robot.model_id, port=isaac_port, on_state=self._on_isaac_state)
        self.isaac_result = None

        self._build_scene(full_meshes)
        self._build_gui()
        self._build_timeline()
        threading.Thread(target=self._ik_loop, daemon=True).start()
        self._load_into_editor(new_project(robot), message='새 프로젝트를 시작했습니다.')

    # ------------------------------------------------------------------ scene
    def _build_scene(self, full_meshes):
        scene = self.server.scene
        scene.set_up_direction('+z')
        scene.add_grid('/ground', width=6., height=6., cell_size=.25, section_size=1., plane='xy')
        self.meshes = [display_mesh(path, full_meshes) for _, _, path, _ in self.robot.visuals]
        self.visual_nodes = []
        self._add_robot_meshes(opacity=.6)
        self.markers = {}
        for key in MARKER_HANDLES:
            marker = scene.add_icosphere(f'/handles/{key}', radius=.03 if key == 'pelvis' else .022,
                                         color=COLORS['normal'], cast_shadow=False)
            marker.on_click(lambda _event, key=key: self.select(key))
            self.markers[key] = marker
        self.selection_label = scene.add_label('/selection_label', '', visible=False, anchor='bottom-center')
        self.com_marker = scene.add_icosphere('/com', radius=.015, color=(40, 200, 90), cast_shadow=False)
        self.com_floor = scene.add_icosphere('/com_floor', radius=.012, color=(40, 200, 90), cast_shadow=False)
        self.gizmo = None
        self.server.on_client_connect(self._frame_camera)

    def _frame_camera(self, client):
        """Start each browser with the robot filling the view (viser's default camera is far away)."""
        client.camera.position = (1.9, -1.6, 1.35)
        client.camera.look_at = (0., 0., .7)
        client.camera.up_direction = (0., 0., 1.)

    def _add_robot_meshes(self, opacity):
        for node in self.visual_nodes:
            node.remove()
        self.visual_nodes = []
        for index, ((vertices, faces), (_, _, _, rgba)) in enumerate(zip(self.meshes, self.robot.visuals)):
            color = tuple(int(255 * c) for c in rgba[:3])
            self.visual_nodes.append(self.server.scene.add_mesh_simple(
                f'/robot/link_{index}', vertices, faces, color=color, opacity=None if opacity >= 1 else opacity))

    def show(self, q):
        state = self.robot.state(q)
        with self.server.atomic():
            for node, (position, wxyz) in zip(self.visual_nodes, state['visuals']):
                node.position, node.wxyz = position, wxyz
            for key, marker in self.markers.items():
                marker.position = state['handles'][key]['position']
            if self.selected is not None:
                self.selection_label.position = np.asarray(state['handles'][self.selected]['position']) + [0, 0, .06]
            self.com_marker.position = state['com']
            self.com_floor.position = (state['com'][0], state['com'][1], 0.)
        return state

    def _refresh_markers(self):
        for key, marker in self.markers.items():
            marker.color = COLORS['selected'] if key == self.selected else COLORS['pinned'] if key in self.pins else COLORS['normal']

    # ----------------------------------------------------------------- status
    def _status(self, text, level='info'):
        self._message = (level, text)
        self._render_status()

    def _render_status(self):
        project = self.project
        frame = project['keyframes'][self.keyframe_index]
        selected = HANDLES[self.selected][2] if self.selected else '없음'
        pinned = ', '.join(HANDLES[k][2] for k in self.pins) or '없음'
        level, text = self._message
        rows = [('모델', f'<b>{html.escape(self.robot.model_id)}</b>'),
                ('프로젝트', html.escape(project['name'] or '(이름 없음)')),
                ('키프레임', f'{self.keyframe_index + 1}/{len(project["keyframes"])} · {html.escape(frame["name"] or "(이름 없음)")}'),
                ('선택', html.escape(selected)), ('위치 고정', html.escape(pinned))]
        table = ''.join(f'<tr><td style="color:#57606a;padding:1px 10px 1px 0;white-space:nowrap">{k}</td><td>{v}</td></tr>'
                        for k, v in rows)
        ik = f'<div style="margin-top:6px;color:#57606a;font-size:12px">{self._ik_summary}</div>' if self._ik_summary else ''
        self.gui_status.content = (
            f'<div style="font-size:13px;line-height:1.5">'
            f'<table style="border-collapse:collapse">{table}</table>{ik}'
            f'<div style="margin-top:8px;padding:6px 8px;border-left:3px solid {MESSAGE_COLORS[level]};'
            f'background:#f6f8fa;color:{MESSAGE_COLORS[level]}">{html.escape(text)}</div></div>')

    # ------------------------------------------------------------------- undo
    def _editor_state(self):
        return {'q': self.q.copy(), 'pins': list(self.pins), 'angle_pins': list(self.angle_pins),
                'keyframes': copy.deepcopy(self.project['keyframes']), 'keyframe_index': self.keyframe_index}

    def _checkpoint(self, coalesce=None):
        """Save the state before an edit. Repeated edits with the same `coalesce` key within 1 s form one step."""
        now = time.monotonic()
        if coalesce is not None and self._last_checkpoint[0] == coalesce and now - self._last_checkpoint[1] < 1.:
            self._last_checkpoint = (coalesce, now)
            return
        with self.lock:
            self.undo_stack.append(self._editor_state())
            del self.undo_stack[:-UNDO_LIMIT]
            self.redo_stack.clear()
        self._last_checkpoint = (coalesce, now)
        self._refresh_undo_buttons()

    def _restore(self, state):
        with self.lock:
            self.q = state['q'].copy()
            self.pins, self.angle_pins = list(state['pins']), list(state['angle_pins'])
            self.project['keyframes'] = copy.deepcopy(state['keyframes'])
            self.keyframe_index = min(state['keyframe_index'], len(self.project['keyframes']) - 1)
            self.motion = None
        self.show(self.q)
        self._sync_joint_sliders()
        self._sync_selection_gui()
        self._refresh_markers()
        self._snap_gizmo()
        self._refresh_keyframe_gui()

    def _undo(self):
        if self.playing or not self.undo_stack:
            return
        with self.lock:
            self.redo_stack.append(self._editor_state())
            state = self.undo_stack.pop()
        self._last_checkpoint = (None, 0.)
        self._restore(state)
        self._refresh_undo_buttons()
        self._status('실행 취소했습니다.')

    def _redo(self):
        if self.playing or not self.redo_stack:
            return
        with self.lock:
            self.undo_stack.append(self._editor_state())
            state = self.redo_stack.pop()
        self._last_checkpoint = (None, 0.)
        self._restore(state)
        self._refresh_undo_buttons()
        self._status('다시 실행했습니다.')

    def _refresh_undo_buttons(self):
        self.gui_undo.disabled = not self.undo_stack
        self.gui_redo.disabled = not self.redo_stack

    # -------------------------------------------------------------- selection
    def select(self, key):
        if self.playing:
            return
        with self.lock:
            self.selected = key
            if self.gizmo is not None:
                self.gizmo.remove()
                self.gizmo = None
            if key is not None:
                handle = self.robot.state(self.q)['handles'][key]
                gizmo = self.server.scene.add_transform_controls(
                    '/gizmo', scale=.2, disable_rotations=key not in ROTATABLE, depth_test=False,
                    position=handle['position'], wxyz=handle['wxyz'])
                gizmo.on_drag_start(lambda _event: self._on_drag_start())
                gizmo.on_update(lambda _event: self._on_drag_update())
                gizmo.on_drag_end(lambda _event: self._on_drag_end())
                self.gizmo = gizmo
                self.selection_label.text = HANDLES[key][2]
                self.selection_label.position = np.asarray(handle['position']) + [0, 0, .06]
            self.selection_label.visible = key is not None
        self._sync_selection_gui()
        self._refresh_markers()
        self._render_status()

    def _snap_gizmo(self):
        if self.gizmo is not None and self.selected is not None:
            handle = self.robot.state(self.q)['handles'][self.selected]
            self.gizmo.position, self.gizmo.wxyz = handle['position'], handle['wxyz']

    def _on_drag_start(self):
        self._checkpoint()
        with self.lock:
            self.anchor = self.q.copy()
            self.drag_start = (np.asarray(self.gizmo.position, dtype=float).copy(), np.asarray(self.gizmo.wxyz, dtype=float).copy())

    def _on_drag_update(self):
        if self.playing or self.selected is None or self.drag_start is None:
            return
        key = self.selected
        if key in self.pins:
            self._status(f'{HANDLES[key][2]}은(는) 위치가 고정되어 있습니다. 이동하려면 고정을 해제하세요.', 'warn')
            return
        position, wxyz = np.asarray(self.gizmo.position, dtype=float), np.asarray(self.gizmo.wxyz, dtype=float)
        request = {}
        if np.linalg.norm(position - self.drag_start[0]) > EPSILON:
            request['selected_targets'] = {key: position.tolist()}
        if key in ROTATABLE and quaternion_angle(wxyz, self.drag_start[1]) > 1e-4:
            request['orientation_targets'] = {key: wxyz_to_xyzw(wxyz).tolist()}
        if request:
            self._submit_ik(request, self.anchor)

    def _on_drag_end(self):
        self.drag_start = None
        self._submit_ik(None, None)  # snap the gizmo after the last pending solve

    # --------------------------------------------------------------------- IK
    def _submit_ik(self, request, anchor):
        """Keep only the newest request so drags never queue stale solves (Motion Creator `pending`)."""
        with self._ik_condition:
            self._ik_request = (request, anchor)
            self._ik_condition.notify()

    def _ik_loop(self):
        while True:
            with self._ik_condition:
                while self._ik_request is None:
                    self._ik_condition.wait()
                request, anchor = self._ik_request
                self._ik_request = None
            if request is None:
                self._snap_gizmo()
                continue
            with self.lock:
                try:
                    q, info = self.robot.solve(self.q, anchor, pins=tuple(self.pins), angle_pins=tuple(self.angle_pins),
                                               mode=self.gui_mode.value, resistance=float(self.gui_resistance.value),
                                               **request)
                except ValueError as error:
                    self._status(str(error), 'error')
                    continue
                if not info['rejected']:
                    self.q = q
            state = self.show(self.q)
            self._sync_joint_sliders()
            self._report(info, state)

    def _report(self, info, state):
        verdict = '고정 유지 불가로 거부' if info['rejected'] else '수렴' if info['converged'] else '근사'
        self._ik_summary = (f'IK {verdict} · 목표 {info["target_error_mm"]:.1f} mm · 각도 {info["angle_error_deg"]:.2f}° · '
                            f'고정 {info["pin_error_mm"]:.2f} mm<br>COM ({state["com"][0]:.3f}, {state["com"][1]:.3f}, '
                            f'{state["com"][2]:.3f}) m · 발바닥 최저 {state["floor_min_mm"]:.1f} mm')
        if info['rejected']:
            self._status('고정된 부위를 지킬 수 없어 이 자세를 적용하지 않았습니다.', 'warn')
        else:
            self._render_status()

    # -------------------------------------------------------------------- GUI
    def _build_gui(self):
        gui = self.server.gui
        gui.configure_theme(control_layout='fixed', control_width='large', brand_color=BRAND, show_share_button=False)
        self.gui_status = gui.add_html('')
        with gui.add_folder('사용법', expand_by_default=False):
            gui.add_markdown(HELP)
        tabs = gui.add_tab_group()

        with tabs.add_tab('편집', icon=viser.Icon.POINTER):
            for row, parts in PART_ROWS.items():
                buttons = gui.add_button_group(row, list(parts))
                buttons.on_click(lambda event, parts=parts: self.select(parts[event.target.value]))
            gui.add_button('선택 해제', icon=viser.Icon.X).on_click(lambda _event: self.select(None))
            with gui.add_folder('선택 부위 고정'):
                self.gui_pin = gui.add_checkbox('위치 고정', False, hint='이 부위의 위치를 IK가 유지합니다. 발은 방향도 함께 유지합니다.')
                self.gui_pin.on_update(by_user(self._on_pin_toggle))
                self.gui_angle_pin = gui.add_checkbox('각도 고정', False, hint='위치는 움직여도 이 부위의 방향 또는 관절각을 유지합니다.')
                self.gui_angle_pin.on_update(by_user(self._on_angle_pin_toggle))
            with gui.add_folder('IK 설정', expand_by_default=False):
                self.gui_mode = gui.add_dropdown('모드', ['elastic', 'free'], initial_value='elastic',
                                                 hint='elastic: 선택 부위에서 먼 관절일수록 원래 자세를 유지합니다. free: 고정만 지킵니다.')
                self.gui_resistance = gui.add_slider('자세 유지 강도', min=.2, max=3., step=.1, initial_value=1.)
            with gui.add_folder('보기', expand_by_default=False):
                self.gui_opacity = gui.add_checkbox('로봇 반투명', True, hint='몸 안쪽의 마커를 보기 쉽게 합니다.')
                self.gui_opacity.on_update(by_user(self._on_opacity_toggle))
                gui.add_button('카메라 초기화', icon=viser.Icon.FOCUS_CENTERED).on_click(
                    lambda _event: [self._frame_camera(c) for c in self.server.get_clients().values()])
            self.gui_undo = gui.add_button('실행 취소', icon=viser.Icon.ARROW_BACK_UP, disabled=True)
            self.gui_undo.on_click(lambda _event: self._undo())
            self.gui_redo = gui.add_button('다시 실행', icon=viser.Icon.ARROW_FORWARD_UP, disabled=True)
            self.gui_redo.on_click(lambda _event: self._redo())

        with tabs.add_tab('키프레임', icon=viser.Icon.KEYFRAMES):
            self.gui_keyframes = gui.add_dropdown('키프레임', ['1. Stand'])
            self.gui_keyframes.on_update(by_user(self._on_keyframe_dropdown))
            gui.add_button_group('이동', ['◀ 이전', '다음 ▶']).on_click(
                lambda event: self._step_keyframe(-1 if event.target.value.startswith('◀') else 1))
            self.gui_kf_name = gui.add_text('이름', initial_value='Stand')
            self.gui_kf_name.on_update(by_user(self._on_keyframe_fields))
            self.gui_kf_duration = gui.add_number('구간 시간 (s)', initial_value=2., min=MIN_DURATION, max=MAX_DURATION, step=.1,
                                                  hint='이전 키프레임에서 이 키프레임까지 걸리는 시간입니다.')
            self.gui_kf_duration.on_update(by_user(self._on_keyframe_fields))
            gui.add_button('현재 자세를 다음 키프레임으로 추가', icon=viser.Icon.PLUS).on_click(lambda _event: self._add_keyframe())
            gui.add_button('현재 자세로 이 키프레임 갱신', icon=viser.Icon.REFRESH).on_click(lambda _event: self._update_keyframe())
            gui.add_button('이 키프레임 자세로 되돌리기', icon=viser.Icon.RESTORE).on_click(
                lambda _event: self._goto_keyframe(self.keyframe_index))
            gui.add_button('이 키프레임 삭제', icon=viser.Icon.TRASH, color='red').on_click(lambda _event: self._delete_keyframe())
            with gui.add_folder('재생'):
                gui.add_button('처음부터 재생', icon=viser.Icon.PLAYER_PLAY, color='green').on_click(lambda _event: self._play())
                gui.add_button('정지', icon=viser.Icon.PLAYER_STOP).on_click(lambda _event: self._stop())
                self.gui_loop = gui.add_checkbox('반복 재생', False)
                self.gui_motion = gui.add_markdown('')

        with tabs.add_tab('관절', icon=viser.Icon.ADJUSTMENTS_HORIZONTAL):
            gui.add_markdown('단위: 도(°). 슬라이더를 움직이면 고정된 부위를 유지하며 관절각을 맞춥니다.')
            self.joint_sliders = {}
            for group, prefixes in JOINT_GROUPS:
                with gui.add_folder(group, expand_by_default=False):
                    for index, name in enumerate(self.robot.names):
                        if not name.startswith(prefixes):
                            continue
                        lower, upper = np.rad2deg(self.robot.lower[3 + index]), np.rad2deg(self.robot.upper[3 + index])
                        slider = gui.add_slider(HANDLES[name][2], min=round(lower, 1), max=round(upper, 1), step=.5,
                                                initial_value=float(np.clip(np.rad2deg(self.q[7 + index]), lower, upper)))
                        slider.on_update(by_user(lambda name=name: self._on_joint_slider(name)))
                        self.joint_sliders[name] = slider

        with tabs.add_tab('검증', icon=viser.Icon.CHECK):
            gui.add_markdown(ISAAC_HELP)
            self.gui_isaac_state = gui.add_markdown('**Isaac 워커**: 꺼짐')
            self.gui_isaac_start = gui.add_button('Isaac 워커 시작', icon=viser.Icon.PLAYER_PLAY)
            self.gui_isaac_start.on_click(lambda _event: self.isaac.start())
            self.gui_isaac_run = gui.add_button('현재 모션을 Isaac WBC로 검증', icon=viser.Icon.CHECK, color='green', disabled=True)
            self.gui_isaac_run.on_click(lambda _event: self._run_isaac())
            self.gui_isaac_progress = gui.add_progress_bar(0., visible=False, animated=True)
            self.gui_isaac_result = gui.add_html('')
            self.gui_isaac_replay = gui.add_button('물리 결과 재생', icon=viser.Icon.PLAYER_PLAY, disabled=True)
            self.gui_isaac_replay.on_click(lambda _event: self._replay_isaac())
            gui.add_button('정지', icon=viser.Icon.PLAYER_STOP).on_click(lambda _event: self._stop())

        with tabs.add_tab('파일', icon=viser.Icon.FOLDER):
            gui.add_markdown(f'모델 **{self.robot.model_id}** · `{self.robot.urdf_path.name}`')
            self.gui_name = gui.add_text('프로젝트 이름', initial_value='')
            self.gui_name.on_update(by_user(self._rename))
            gui.add_button('저장', icon=viser.Icon.DEVICE_FLOPPY, color='green').on_click(lambda _event: self._save(save_as=False))
            gui.add_button('다른 이름으로 저장', icon=viser.Icon.COPY).on_click(lambda _event: self._save(save_as=True))
            gui.add_button('새 프로젝트', icon=viser.Icon.FILE_PLUS).on_click(
                lambda _event: self._load_into_editor(new_project(self.robot), message='새 프로젝트를 시작했습니다.'))
            with gui.add_folder('열기'):
                self.gui_saved = gui.add_dropdown('저장된 프로젝트', self._saved_options())
                gui.add_button('열기', icon=viser.Icon.FOLDER_OPEN).on_click(lambda _event: self._open_saved())
                gui.add_button('목록 새로고침', icon=viser.Icon.REFRESH).on_click(lambda _event: self._refresh_saved())
                upload = gui.add_upload_button('JSON 파일 불러오기', icon=viser.Icon.UPLOAD, mime_type='application/json',
                                               hint='motion-sim 또는 Motion Creator의 project.json')
                upload.on_upload(lambda _event: self._open_upload(upload.value))

    def _sync_selection_gui(self):
        key = self.selected
        pinned, angle_pinned = key in self.pins, key in self.angle_pins
        if self.gui_pin.value != pinned:
            self.gui_pin.value = pinned
        if self.gui_angle_pin.value != angle_pinned:
            self.gui_angle_pin.value = angle_pinned
        self.gui_pin.disabled = key is None
        self.gui_angle_pin.disabled = key not in ANGLE_LOCKABLE

    def _on_pin_toggle(self):
        key = self.selected
        if key is None or self.gui_pin.value == (key in self.pins):
            return
        self._checkpoint()
        with self.lock:
            if self.gui_pin.value:
                self.pins.append(key)
            else:
                self.pins.remove(key)
        self._refresh_markers()
        self._render_status()

    def _on_angle_pin_toggle(self):
        key = self.selected
        if key is None or key not in ANGLE_LOCKABLE or self.gui_angle_pin.value == (key in self.angle_pins):
            return
        self._checkpoint()
        with self.lock:
            if self.gui_angle_pin.value:
                self.angle_pins.append(key)
            else:
                self.angle_pins.remove(key)

    def _on_opacity_toggle(self):
        self._add_robot_meshes(.6 if self.gui_opacity.value else 1.)
        self.show(self.q)

    def _sync_joint_sliders(self):
        for index, name in enumerate(self.robot.names):
            value = round(float(np.rad2deg(self.q[7 + index])), 3)
            slider = self.joint_sliders[name]
            if abs(slider.value - value) > 1e-3:
                slider.value = value

    def _on_joint_slider(self, name):
        if self.playing:
            return
        index = self.robot.names.index(name)
        target = float(np.deg2rad(self.joint_sliders[name].value))
        if abs(target - self.q[7 + index]) < np.deg2rad(.01):
            return
        self._checkpoint(coalesce=f'joint:{name}')
        target = float(np.clip(target, self.robot.lower[3 + index], self.robot.upper[3 + index]))
        self._submit_ik({'joint_targets': {name: target}}, self.q.copy())

    # ---------------------------------------------------------------- project
    def _load_into_editor(self, project, warnings=(), message='프로젝트를 불러왔습니다.'):
        self._stop()
        with self.lock:
            self.project = project
            self.q = self.robot.validate_q(project.get('current_qpos') or project['keyframes'][0]['qpos'])
            self.pins = list(project.get('pins', FEET))
            self.angle_pins = list(project.get('angle_pins', []))
            self.keyframe_index = 0
            self.motion = None
            self.undo_stack.clear()
            self.redo_stack.clear()
        if self.gui_name.value != project['name']:
            self.gui_name.value = project['name']
        self._ik_summary = ''
        self.select(None)
        self.show(self.q)
        self._sync_joint_sliders()
        self._refresh_keyframe_gui()
        self._refresh_undo_buttons()
        self._status(' '.join([message, *warnings]), 'warn' if warnings else 'ok')

    def _rename(self):
        name = self.gui_name.value[:80]
        if name != self.project['name']:
            self.project['name'] = name
            self._render_status()

    def _snapshot(self):
        with self.lock:
            self.project['current_qpos'] = self.q.tolist()
            self.project['pins'], self.project['angle_pins'] = list(self.pins), list(self.angle_pins)
            return self.project

    def _save(self, save_as):
        try:
            folder, saved = save_bundle(self.robot, self._snapshot(), FPS, self.motions_dir, save_as=save_as)
        except ValueError as error:
            self._status(f'저장하지 못했습니다: {error}', 'error')
            return
        with self.lock:
            self.project['project_id'], self.project['created_at'] = saved['project_id'], saved['created_at']
        self._refresh_saved(select=folder.name)
        self._status(f'저장했습니다: {folder}', 'ok')

    def _saved_options(self):
        self.saved_folders = {folder.name: folder for folder, _ in list_saved(self.motions_dir)}
        return list(self.saved_folders) or ['(저장된 프로젝트 없음)']

    def _refresh_saved(self, select=None):
        self.gui_saved.options = self._saved_options()
        if select in self.saved_folders:
            self.gui_saved.value = select

    def _open_saved(self):
        folder = self.saved_folders.get(self.gui_saved.value)
        if folder is not None:
            self._open_text((folder / 'project.json').read_text())

    def _open_upload(self, uploaded):
        self._open_text(uploaded.content.decode('utf-8'))

    def _open_text(self, text):
        try:
            project, warnings = load_project(self.robot, text)
        except (ValueError, KeyError, TypeError) as error:
            self._status(f'불러오지 못했습니다: {error}', 'error')
            return
        self._load_into_editor(project, warnings)

    # -------------------------------------------------------------- keyframes
    def _keyframe_options(self):
        return [f'{index + 1}. {frame["name"] or "(이름 없음)"}' for index, frame in enumerate(self.project['keyframes'])]

    def _refresh_keyframe_gui(self):
        options = self._keyframe_options()
        self.gui_keyframes.options = options
        self.gui_keyframes.value = options[self.keyframe_index]
        frame = self.project['keyframes'][self.keyframe_index]
        if self.gui_kf_name.value != frame['name']:
            self.gui_kf_name.value = frame['name']
        if abs(self.gui_kf_duration.value - frame['duration']) > EPSILON:
            self.gui_kf_duration.value = frame['duration']
        self.gui_kf_duration.disabled = self.keyframe_index == 0
        times = keyframe_times(self.project)
        self.gui_motion.content = f'키프레임 {len(options)}개 · 전체 {times[-1]:.2f}초'
        self._refresh_timeline()
        self._render_status()

    def _changed(self):
        self.motion = None
        self._refresh_keyframe_gui()

    def _on_keyframe_dropdown(self):
        options = self._keyframe_options()
        if self.gui_keyframes.value in options and options.index(self.gui_keyframes.value) != self.keyframe_index:
            self._goto_keyframe(options.index(self.gui_keyframes.value))

    def _step_keyframe(self, step):
        index = self.keyframe_index + step
        if 0 <= index < len(self.project['keyframes']):
            self._goto_keyframe(index)

    def _goto_keyframe(self, index):
        if self.playing:
            return
        self._checkpoint(coalesce='goto')
        with self.lock:
            self.keyframe_index = index
            frame = self.project['keyframes'][index]
            self.q = self.robot.validate_q(frame['qpos'])
            self.pins, self.angle_pins = list(frame.get('pins', [])), list(frame.get('angle_pins', []))
        self.show(self.q)
        self._sync_joint_sliders()
        self._sync_selection_gui()
        self._refresh_markers()
        self._snap_gizmo()
        self._refresh_keyframe_gui()
        self.server.timeline.set_current_frame(round(keyframe_times(self.project)[index] * FPS))

    def _on_keyframe_fields(self):
        frame = self.project['keyframes'][self.keyframe_index]
        name, duration = self.gui_kf_name.value[:80], float(self.gui_kf_duration.value)
        if name == frame['name'] and abs(duration - frame['duration']) < EPSILON:
            return
        if not MIN_DURATION <= duration <= MAX_DURATION:
            self._status(f'구간 시간은 {MIN_DURATION:g}–{MAX_DURATION:g}초여야 합니다.', 'warn')
            return
        self._checkpoint(coalesce=f'fields:{self.keyframe_index}')
        with self.lock:
            frame['name'], frame['duration'] = name, duration
        self._changed()

    def _current_keyframe(self, name, duration):
        return {'name': name, 'duration': duration, 'qpos': self.q.tolist(),
                'pins': list(self.pins), 'angle_pins': list(self.angle_pins)}

    def _add_keyframe(self, index=None, duration=1.):
        if self.playing:
            return
        self._checkpoint()
        with self.lock:
            index = self.keyframe_index + 1 if index is None else index
            self.project['keyframes'].insert(index, self._current_keyframe(f'Key {len(self.project["keyframes"]) + 1}', duration))
            self.keyframe_index = index
        self._changed()
        self._status(f'{index + 1}번 키프레임을 추가했습니다.', 'ok')

    def _update_keyframe(self):
        if self.playing:
            return
        self._checkpoint()
        with self.lock:
            frame = self.project['keyframes'][self.keyframe_index]
            frame.update(self._current_keyframe(frame['name'], frame['duration']))
        self._changed()
        self._status(f'{self.keyframe_index + 1}번 키프레임을 현재 자세로 갱신했습니다.', 'ok')

    def _delete_keyframe(self, index=None):
        if self.playing:
            return
        if len(self.project['keyframes']) == 1:
            self._status('마지막 키프레임은 삭제할 수 없습니다.', 'warn')
            self._refresh_timeline()
            return
        self._checkpoint()
        with self.lock:
            frames = self.project['keyframes']
            index = self.keyframe_index if index is None else index
            frames.pop(index)
            if index == 0:
                frames[0]['duration'] = 2.
            self.keyframe_index = min(index, len(frames) - 1)
        self._changed()
        self._status(f'{index + 1}번 키프레임을 삭제했습니다. 실행 취소로 되돌릴 수 있습니다.', 'ok')

    # --------------------------------------------------------------- timeline
    def _build_timeline(self):
        timeline = self.server.timeline
        timeline.set_visible(True)
        timeline.set_fps(FPS)
        self.track = timeline.add_track('키프레임', track_type='keyframe', color=(255, 196, 30), uuid='keyframes')
        timeline.on_keyframe_move(self._on_timeline_move)
        timeline.on_keyframe_add(self._on_timeline_add)
        timeline.on_keyframe_delete(self._on_timeline_delete)
        timeline.on_frame_change(self._on_timeline_frame)

    def _refresh_timeline(self):
        timeline = self.server.timeline
        times = keyframe_times(self.project)
        timeline.clear_keyframes(self.track)
        self.timeline_ids = []
        for index, seconds in enumerate(times):
            frame = round(seconds * FPS)
            if index == 0:
                self.timeline_ids.append(timeline.add_locked_keyframe(self.track, frame))
            else:
                self.timeline_ids.append(timeline.add_keyframe(self.track, frame))
        timeline.set_frame_range(0, round(times[-1] * FPS) + FPS)

    def _on_timeline_move(self, keyframe_id, new_frame):
        if keyframe_id not in self.timeline_ids or self.playing:
            return
        index = self.timeline_ids.index(keyframe_id)
        times = keyframe_times(self.project)
        seconds = new_frame / FPS
        frames = self.project['keyframes']
        upper = times[index + 1] - MIN_DURATION if index + 1 < len(times) else times[index - 1] + MAX_DURATION
        if index == 0 or not times[index - 1] + MIN_DURATION <= seconds <= upper:
            self._refresh_timeline()  # revert: keyframes cannot pass their neighbours
            self._status('키프레임은 이웃 키프레임을 넘어갈 수 없습니다.', 'warn')
            return
        self._checkpoint()
        with self.lock:
            frames[index]['duration'] = round(seconds - times[index - 1], 4)
            if index + 1 < len(frames):
                frames[index + 1]['duration'] = round(times[index + 1] - seconds, 4)
            self.keyframe_index = index
        self._changed()

    def _on_timeline_add(self, keyframe_id, track_id, frame):
        if self.playing:
            return
        times = keyframe_times(self.project)
        seconds = frame / FPS
        index = int(np.searchsorted(times, seconds))
        too_close = any(abs(seconds - t) < MIN_DURATION for t in times) or seconds - times[-1] > MAX_DURATION
        if index == 0 or too_close:
            self._refresh_timeline()
            return
        self._checkpoint()
        with self.lock:
            frames = self.project['keyframes']
            if index < len(frames):
                frames[index]['duration'] = round(times[index] - seconds, 4)
            frames.insert(index, self._current_keyframe(f'Key {len(frames) + 1}', round(seconds - times[index - 1], 4)))
            self.keyframe_index = index
        self._changed()

    def _on_timeline_delete(self, keyframe_id):
        if keyframe_id in self.timeline_ids:
            self._delete_keyframe(self.timeline_ids.index(keyframe_id))

    def _on_timeline_frame(self, frame):
        if self.playing:
            return
        times = [round(t * FPS) for t in keyframe_times(self.project)]
        if frame in times:
            self._goto_keyframe(times.index(frame))
            return
        motion = self._compiled()
        if motion is None:
            return
        self._checkpoint(coalesce='scrub')
        with self.lock:
            self.q = motion['qpos'][int(np.clip(frame, 0, len(motion['qpos']) - 1))].copy()
        self.show(self.q)
        self._sync_joint_sliders()
        self._snap_gizmo()
        self._status(f'{frame / FPS:.2f}초의 보간 자세입니다. 키프레임으로 추가하거나 여기서 편집할 수 있습니다.')

    # --------------------------------------------------------------- playback
    def _compiled(self):
        if self.motion is None:
            try:
                self.motion = compile_motion(self.robot, self._snapshot(), FPS)
            except ValueError as error:
                self._status(f'모션을 만들 수 없습니다: {error}', 'error')
                return None
            self.gui_motion.content = (f'모션 {self.motion["time"][-1]:.2f}초 · {len(self.motion["time"])}프레임 · '
                                       f'최대 고정 오차 {self.motion["max_pin_error_mm"]:.2f} mm (기구학)')
        return self.motion

    def _play(self):
        if self.playing:
            return
        motion = self._compiled()
        if motion is None:
            return
        self.select(None)
        self.playing = True
        self._status('재생 중입니다. 재생 중에는 편집할 수 없습니다.')
        threading.Thread(target=self._play_loop, args=(motion,), daemon=True).start()

    def _play_loop(self, motion):
        while self.playing:
            start = time.perf_counter()
            for index, q in enumerate(motion['qpos']):
                if not self.playing:
                    break
                self.show(q)
                self.server.timeline.set_current_frame(index)
                delay = start + (index + 1) / FPS - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
            if not self.gui_loop.value:
                break
        self.playing = False
        self.show(self.q)
        self._status('재생을 마쳤습니다.')

    def _stop(self):
        self.playing = False

    # ------------------------------------------------------------ Isaac check
    def _on_isaac_state(self, state, detail):
        labels = {'stopped': '꺼짐', 'starting': '시작 중', 'ready': '준비됨', 'running': '계산 중', 'failed': '실패'}
        self.gui_isaac_state.content = f'**Isaac 워커**: {labels.get(state, state)}' + (f' · {detail}' if detail else '')
        self.gui_isaac_start.disabled = state in ('starting', 'ready', 'running')
        self.gui_isaac_run.disabled = state != 'ready'
        if state == 'failed':
            self._status(detail, 'error')

    def _run_isaac(self):
        if self.playing or self.isaac.state != 'ready':
            return
        project = copy.deepcopy(self._snapshot())
        try:
            compile_motion(self.robot, copy.deepcopy(project), FPS)   # surface authoring errors before Isaac
        except ValueError as error:
            self._status(f'모션을 만들 수 없어 검증하지 않았습니다: {error}', 'error')
            return
        self.gui_isaac_progress.value, self.gui_isaac_progress.visible = 0., True
        self.gui_isaac_replay.disabled = True
        self._status('Isaac에서 WBC 추종을 계산하고 있습니다.')
        threading.Thread(target=self._isaac_job, args=(project,), daemon=True).start()

    def _isaac_job(self, project):
        try:
            result = self.isaac.run(project, lambda value: setattr(self.gui_isaac_progress, 'value', round(100 * value, 1)))
        except RuntimeError as error:
            self._status(str(error), 'error')
            return
        finally:
            self.gui_isaac_progress.visible = False
        if result['type'] != 'result':
            self._status(f'Isaac 검증 실패: {result.get("message")}', 'error')
            return
        self.isaac_result = result
        self.gui_isaac_result.content = self._isaac_report(result)
        self.gui_isaac_replay.disabled = False
        summary = result['summary']
        if summary['fell']:
            self._status('Isaac 검증: 로봇이 넘어졌습니다. 결과를 재생해 넘어지는 지점을 확인하세요.', 'error')
        else:
            self._status('Isaac 검증: 넘어지지 않고 모션을 끝까지 추종했습니다.', 'ok')

    def _isaac_report(self, result):
        s = result['summary']
        moved = max(0., s['seconds'] - s['settle_seconds'])
        verdict = (f'<b style="color:{MESSAGE_COLORS["error"]}">낙상</b> · 준비 후 {moved:.2f}초 지점'
                   if s['fell'] else f'<b style="color:{MESSAGE_COLORS["ok"]}">완주</b> · 모션 {s["motion_seconds"]:.2f}초 + 유지')
        rows = [('결과', verdict),
                ('팔 추종', f'RMSE {s["upper_rmse_deg"]:.2f}° · 최대 {s["upper_max_deg"]:.2f}°'),
                ('다리·허리', f'RMSE {s["lower_rmse_deg"]:.2f}° (정책이 걸음을 만들므로 저작 자세와 다를 수 있음)'),
                ('루트 위치', f'최대 오차 {s.get("max_root_error_m", 0.):.3f} m · yaw 최대 오차 {s.get("max_yaw_error_deg", 0.):.1f}°'),
                ('조건', f'Isaac Sim 물리 {1 / result["settings"]["physics_dt"]:.0f} Hz · 정책 50 Hz · 모델 {html.escape(self.robot.model_id)}')]
        table = ''.join(f'<tr><td style="color:#57606a;padding:2px 10px 2px 0;white-space:nowrap;vertical-align:top">{k}</td>'
                        f'<td>{v}</td></tr>' for k, v in rows)
        return f'<div style="font-size:13px;line-height:1.5"><table style="border-collapse:collapse">{table}</table></div>'

    def _replay_isaac(self):
        if self.playing or self.isaac_result is None:
            return
        self.select(None)
        self.playing = True
        self._status('Isaac 물리 결과를 재생합니다 (준비 2초 포함).')
        threading.Thread(target=self._replay_loop, args=(self.isaac_result['frames'],), daemon=True).start()

    def _replay_loop(self, frames):
        start = time.perf_counter()
        for frame in frames:
            if not self.playing:
                break
            self.show(np.asarray(frame['qpos']))
            self.server.timeline.set_current_frame(round(frame['motion_time'] * FPS))
            delay = start + frame['time'] - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
        self.playing = False
        self.show(self.q)
        self._status('Isaac 결과 재생을 마쳤습니다.')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', default='g1', choices=['g1', 'g1-tools'])
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--motions-dir', type=Path, default=DEFAULT_MOTIONS_DIR)
    parser.add_argument('--full-meshes', action='store_true', help='표시용 메시를 줄이지 않고 원본 그대로 보냅니다')
    parser.add_argument('--isaac', action='store_true', help='시작할 때 Isaac 워커도 함께 띄웁니다')
    parser.add_argument('--isaac-port', type=int, default=8091)
    args = parser.parse_args()
    server = viser.ViserServer(host=args.host, port=args.port, label='motion-sim')
    editor = Editor(server, Robot(args.model), args.motions_dir, full_meshes=args.full_meshes, isaac_port=args.isaac_port)
    if args.isaac:
        editor.isaac.start()
    # viser swallows the default SIGINT handling; stop the Isaac worker explicitly on shutdown signals.
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: sys.exit(0))
    try:
        while True:
            time.sleep(3600)
    finally:
        editor.isaac.stop()


if __name__ == '__main__':
    main()

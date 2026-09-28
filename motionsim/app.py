"""viser kinematic motion editor.

Run on the GPU server and open http://<server>:8080 in a local browser:
    python -m motionsim.app --model g1 --port 8080
"""
import argparse
import threading
import time
from pathlib import Path

import numpy as np
import trimesh
import viser

from .motion import (DEFAULT_MOTIONS_DIR, MAX_DURATION, MIN_DURATION, compile_motion, keyframe_times,
                     list_saved, load_project, new_project, save_bundle)
from .robot import ANGLE_LOCKABLE, FEET, HANDLES, ROTATABLE, Robot

FPS = 30
# Clickable markers. Individual joints are edited with the joint sliders instead.
MARKER_HANDLES = ('pelvis', 'waist', 'left_hip', 'right_hip', 'left_knee', 'right_knee', 'left_foot', 'right_foot',
                  'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow', 'left_hand', 'right_hand')
COLORS = {'normal': (60, 130, 255), 'selected': (255, 196, 30), 'pinned': (230, 60, 60)}
NO_SELECTION = '(선택 없음)'
EPSILON = 1e-6


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


class Editor:
    def __init__(self, server: viser.ViserServer, robot: Robot, motions_dir: Path):
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
        self._ik_request = None
        self._ik_condition = threading.Condition()
        self.labels = {HANDLES[key][2]: key for key in MARKER_HANDLES}

        self._build_scene()
        self._build_gui()
        self._build_timeline()
        threading.Thread(target=self._ik_loop, daemon=True).start()
        self._load_into_editor(self.project)

    # ------------------------------------------------------------------ scene
    def _build_scene(self):
        scene = self.server.scene
        scene.set_up_direction('+z')
        scene.add_grid('/ground', width=6., height=6., cell_size=.25, section_size=1., plane='xy')
        self.meshes = [trimesh.load(path, force='mesh') for _, _, path, _ in self.robot.visuals]
        self.visual_nodes = []
        self._add_robot_meshes(opacity=.6)
        self.markers = {}
        for key in MARKER_HANDLES:
            marker = scene.add_icosphere(f'/handles/{key}', radius=.03 if key == 'pelvis' else .022,
                                         color=COLORS['normal'], cast_shadow=False)
            marker.on_click(lambda _event, key=key: self.select(key))
            self.markers[key] = marker
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
        for index, (mesh, (_, _, _, rgba)) in enumerate(zip(self.meshes, self.robot.visuals)):
            color = tuple(int(255 * c) for c in rgba[:3])
            self.visual_nodes.append(self.server.scene.add_mesh_simple(
                f'/robot/link_{index}', mesh.vertices, mesh.faces, color=color,
                opacity=None if opacity >= 1 else opacity))

    def show(self, q):
        state = self.robot.state(q)
        with self.server.atomic():
            for node, (position, wxyz) in zip(self.visual_nodes, state['visuals']):
                node.position, node.wxyz = position, wxyz
            for key, marker in self.markers.items():
                marker.position = state['handles'][key]['position']
            self.com_marker.position = state['com']
            self.com_floor.position = (state['com'][0], state['com'][1], 0.)
        return state

    def _refresh_markers(self):
        for key, marker in self.markers.items():
            marker.color = COLORS['selected'] if key == self.selected else COLORS['pinned'] if key in self.pins else COLORS['normal']

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
        self._sync_selection_gui()
        self._refresh_markers()

    def _snap_gizmo(self):
        if self.gizmo is not None and self.selected is not None:
            handle = self.robot.state(self.q)['handles'][self.selected]
            self.gizmo.position, self.gizmo.wxyz = handle['position'], handle['wxyz']

    def _on_drag_start(self):
        with self.lock:
            self.anchor = self.q.copy()
            self.drag_start = (np.asarray(self.gizmo.position, dtype=float).copy(), np.asarray(self.gizmo.wxyz, dtype=float).copy())

    def _on_drag_update(self):
        if self.playing or self.selected is None or self.drag_start is None:
            return
        key = self.selected
        if key in self.pins:
            self._status(f'{HANDLES[key][2]}은(는) 위치가 고정되어 있습니다. 이동하려면 고정을 해제하세요.')
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
                    self._status(str(error))
                    continue
                if not info['rejected']:
                    self.q = q
            state = self.show(self.q)
            self._sync_joint_sliders()
            self._report(info, state)

    def _report(self, info, state):
        verdict = '거부됨(고정 유지 불가)' if info['rejected'] else '수렴' if info['converged'] else '근사'
        self.gui_ik.content = (f'**IK** {verdict} · 목표 오차 {info["target_error_mm"]:.1f} mm · '
                               f'각도 오차 {info["angle_error_deg"]:.2f}° · 고정 오차 {info["pin_error_mm"]:.2f} mm · '
                               f'평가 {info["evaluations"]}회\n\n'
                               f'COM ({state["com"][0]:.3f}, {state["com"][1]:.3f}, {state["com"][2]:.3f}) m · '
                               f'발바닥 최저 {state["floor_min_mm"]:.1f} mm')

    # -------------------------------------------------------------------- GUI
    def _build_gui(self):
        gui = self.server.gui
        with gui.add_folder('프로젝트'):
            gui.add_markdown(f'모델: **{self.robot.model_id}** · URDF `{self.robot.urdf_path.name}`')
            self.gui_name = gui.add_text('이름', initial_value='')
            self.gui_name.on_update(by_user(self._rename))
            gui.add_button('새 프로젝트').on_click(lambda _event: self._load_into_editor(new_project(self.robot)))
            gui.add_button('저장').on_click(lambda _event: self._save(save_as=False))
            gui.add_button('다른 이름으로 저장').on_click(lambda _event: self._save(save_as=True))
            self.gui_saved = gui.add_dropdown('저장된 프로젝트', self._saved_options())
            gui.add_button('열기').on_click(lambda _event: self._open_saved())
            gui.add_button('목록 새로고침').on_click(lambda _event: self._refresh_saved())
            upload = gui.add_upload_button('JSON 불러오기 (Motion Creator 포함)', mime_type='application/json')
            upload.on_upload(lambda _event: self._open_upload(upload.value))

        with gui.add_folder('선택 부위'):
            self.gui_handle = gui.add_dropdown('부위', [NO_SELECTION, *self.labels])
            self.gui_handle.on_update(by_user(self._on_handle_dropdown))
            self.gui_pin = gui.add_checkbox('위치 고정', False)
            self.gui_pin.on_update(by_user(self._on_pin_toggle))
            self.gui_angle_pin = gui.add_checkbox('각도 고정', False)
            self.gui_angle_pin.on_update(by_user(self._on_angle_pin_toggle))
            self.gui_mode = gui.add_dropdown('IK 모드', ['elastic', 'free'], initial_value='elastic')
            self.gui_resistance = gui.add_slider('자세 유지 강도', min=.2, max=3., step=.1, initial_value=1.)
            self.gui_opacity = gui.add_checkbox('로봇 반투명', True)
            self.gui_opacity.on_update(by_user(self._on_opacity_toggle))
            self.gui_ik = gui.add_markdown('')

        with gui.add_folder('관절 (°)', expand_by_default=False):
            self.joint_sliders = {}
            for index, name in enumerate(self.robot.names):
                lower, upper = np.rad2deg(self.robot.lower[3 + index]), np.rad2deg(self.robot.upper[3 + index])
                slider = gui.add_slider(HANDLES[name][2], min=round(lower, 1), max=round(upper, 1), step=.5,
                                        initial_value=float(np.clip(np.rad2deg(self.q[7 + index]), lower, upper)))
                slider.on_update(by_user(lambda name=name: self._on_joint_slider(name)))
                self.joint_sliders[name] = slider

        with gui.add_folder('키프레임'):
            self.gui_keyframes = gui.add_dropdown('키프레임', ['1. Stand'])
            self.gui_keyframes.on_update(by_user(self._on_keyframe_dropdown))
            self.gui_kf_name = gui.add_text('이름', initial_value='Stand')
            self.gui_kf_name.on_update(by_user(self._on_keyframe_fields))
            self.gui_kf_duration = gui.add_number('이전 키프레임에서 걸리는 시간 (s)', initial_value=2.,
                                                  min=MIN_DURATION, max=MAX_DURATION, step=.1)
            self.gui_kf_duration.on_update(by_user(self._on_keyframe_fields))
            gui.add_button('현재 자세로 추가 (선택 뒤)').on_click(lambda _event: self._add_keyframe())
            gui.add_button('현재 자세로 갱신').on_click(lambda _event: self._update_keyframe())
            gui.add_button('키프레임 자세 불러오기').on_click(lambda _event: self._goto_keyframe(self.keyframe_index))
            gui.add_button('삭제', color='red').on_click(lambda _event: self._delete_keyframe())

        with gui.add_folder('재생'):
            gui.add_button('재생').on_click(lambda _event: self._play())
            gui.add_button('정지').on_click(lambda _event: self._stop())
            self.gui_status = gui.add_markdown('')

    def _status(self, text):
        self.gui_status.content = text

    def _on_opacity_toggle(self):
        self._add_robot_meshes(.6 if self.gui_opacity.value else 1.)
        self.show(self.q)

    def _sync_selection_gui(self):
        key = self.selected
        label = HANDLES[key][2] if key in MARKER_HANDLES else NO_SELECTION
        if self.gui_handle.value != label:
            self.gui_handle.value = label
        pinned, angle_pinned = key in self.pins, key in self.angle_pins
        if self.gui_pin.value != pinned:
            self.gui_pin.value = pinned
        if self.gui_angle_pin.value != angle_pinned:
            self.gui_angle_pin.value = angle_pinned
        self.gui_angle_pin.disabled = key not in ANGLE_LOCKABLE

    def _on_handle_dropdown(self):
        key = self.labels.get(self.gui_handle.value)
        if key != self.selected:
            self.select(key)

    def _on_pin_toggle(self):
        key = self.selected
        if key is None or self.gui_pin.value == (key in self.pins):
            return
        with self.lock:
            if self.gui_pin.value:
                self.pins.append(key)
            else:
                self.pins.remove(key)
        self._refresh_markers()

    def _on_angle_pin_toggle(self):
        key = self.selected
        if key is None or key not in ANGLE_LOCKABLE or self.gui_angle_pin.value == (key in self.angle_pins):
            return
        with self.lock:
            if self.gui_angle_pin.value:
                self.angle_pins.append(key)
            else:
                self.angle_pins.remove(key)

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
        # Programmatic syncs write the current angle; only a real change requests IK.
        if abs(target - self.q[7 + index]) < np.deg2rad(.01):
            return
        target = float(np.clip(target, self.robot.lower[3 + index], self.robot.upper[3 + index]))
        self._submit_ik({'joint_targets': {name: target}}, self.q.copy())

    # ---------------------------------------------------------------- project
    def _load_into_editor(self, project, warnings=()):
        self._stop()
        with self.lock:
            self.project = project
            self.q = self.robot.validate_q(project.get('current_qpos') or project['keyframes'][0]['qpos'])
            self.pins = list(project.get('pins', FEET))
            self.angle_pins = list(project.get('angle_pins', []))
            self.keyframe_index = 0
            self.motion = None
        if self.gui_name.value != project['name']:
            self.gui_name.value = project['name']
        self.select(None)
        self.show(self.q)
        self._sync_joint_sliders()
        self._refresh_keyframe_gui()
        self._status('\n\n'.join(['프로젝트를 불러왔습니다.', *warnings]))

    def _rename(self):
        name = self.gui_name.value[:80]
        if name != self.project['name']:
            self.project['name'] = name

    def _snapshot(self):
        with self.lock:
            self.project['current_qpos'] = self.q.tolist()
            self.project['pins'], self.project['angle_pins'] = list(self.pins), list(self.angle_pins)
            return self.project

    def _save(self, save_as):
        try:
            folder, saved = save_bundle(self.robot, self._snapshot(), FPS, self.motions_dir, save_as=save_as)
        except ValueError as error:
            self._status(f'저장 실패: {error}')
            return
        with self.lock:
            self.project['project_id'], self.project['created_at'] = saved['project_id'], saved['created_at']
        self._refresh_saved()
        self._status(f'저장했습니다: `{folder}`')

    def _saved_options(self):
        self.saved_folders = {f'{folder.name}': folder for folder, _ in list_saved(self.motions_dir)}
        return list(self.saved_folders) or ['(저장된 프로젝트 없음)']

    def _refresh_saved(self):
        self.gui_saved.options = self._saved_options()

    def _open_saved(self):
        folder = self.saved_folders.get(self.gui_saved.value)
        if folder is None:
            return
        self._open_text((folder / 'project.json').read_text())

    def _open_upload(self, uploaded):
        self._open_text(uploaded.content.decode('utf-8'))

    def _open_text(self, text):
        try:
            project, warnings = load_project(self.robot, text)
        except (ValueError, KeyError, TypeError) as error:
            self._status(f'불러오기 실패: {error}')
            return
        self._load_into_editor(project, warnings)

    # -------------------------------------------------------------- keyframes
    def _keyframe_options(self):
        return [f'{index + 1}. {frame["name"] or "(이름 없음)"}' for index, frame in enumerate(self.project['keyframes'])]

    def _refresh_keyframe_gui(self):
        self.gui_keyframes.options = self._keyframe_options()
        self.gui_keyframes.value = self._keyframe_options()[self.keyframe_index]
        frame = self.project['keyframes'][self.keyframe_index]
        if self.gui_kf_name.value != frame['name']:
            self.gui_kf_name.value = frame['name']
        if abs(self.gui_kf_duration.value - frame['duration']) > EPSILON:
            self.gui_kf_duration.value = frame['duration']
        self.gui_kf_duration.disabled = self.keyframe_index == 0
        self._refresh_timeline()

    def _changed(self):
        self.motion = None
        self._refresh_keyframe_gui()

    def _on_keyframe_dropdown(self):
        options = self._keyframe_options()
        if self.gui_keyframes.value not in options:
            return
        index = options.index(self.gui_keyframes.value)
        if index != self.keyframe_index:
            self._goto_keyframe(index)

    def _goto_keyframe(self, index):
        if self.playing:
            return
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
            self._status(f'duration은 {MIN_DURATION:g}–{MAX_DURATION:g}초여야 합니다')
            return
        with self.lock:
            frame['name'], frame['duration'] = name, duration
        self._changed()

    def _current_keyframe(self, name, duration):
        return {'name': name, 'duration': duration, 'qpos': self.q.tolist(),
                'pins': list(self.pins), 'angle_pins': list(self.angle_pins)}

    def _add_keyframe(self, index=None, duration=1.):
        if self.playing:
            return
        with self.lock:
            index = self.keyframe_index + 1 if index is None else index
            self.project['keyframes'].insert(index, self._current_keyframe(f'Key {len(self.project["keyframes"]) + 1}', duration))
            self.keyframe_index = index
        self._changed()

    def _update_keyframe(self):
        if self.playing:
            return
        with self.lock:
            frame = self.project['keyframes'][self.keyframe_index]
            frame.update(self._current_keyframe(frame['name'], frame['duration']))
        self._changed()
        self._status(f'{self.keyframe_index + 1}번 키프레임을 현재 자세로 갱신했습니다.')

    def _delete_keyframe(self, index=None):
        if self.playing:
            return
        with self.lock:
            frames = self.project['keyframes']
            if len(frames) == 1:
                self._status('마지막 키프레임은 삭제할 수 없습니다.')
                self._refresh_timeline()
                return
            index = self.keyframe_index if index is None else index
            frames.pop(index)
            if index == 0:
                frames[0]['duration'] = 2.
            self.keyframe_index = min(index, len(frames) - 1)
        self._changed()

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
            return
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
        with self.lock:
            frames = self.project['keyframes']
            if index < len(frames):
                frames[index]['duration'] = round(times[index] - seconds, 4)
            self.project['keyframes'].insert(index, self._current_keyframe(f'Key {len(frames) + 1}', round(seconds - times[index - 1], 4)))
            self.keyframe_index = index
        self._changed()

    def _on_timeline_delete(self, keyframe_id):
        if keyframe_id in self.timeline_ids:
            self._delete_keyframe(self.timeline_ids.index(keyframe_id))

    def _on_timeline_frame(self, frame):
        if self.playing:
            return
        motion = self._compiled()
        if motion is None:
            return
        index = int(np.clip(frame, 0, len(motion['qpos']) - 1))
        times = [round(t * FPS) for t in keyframe_times(self.project)]
        if frame in times:
            self._goto_keyframe(times.index(frame))
            return
        with self.lock:
            self.q = motion['qpos'][index].copy()
        self.show(self.q)
        self._sync_joint_sliders()
        self._snap_gizmo()

    # --------------------------------------------------------------- playback
    def _compiled(self):
        if self.motion is None:
            try:
                self.motion = compile_motion(self.robot, self._snapshot(), FPS)
            except ValueError as error:
                self._status(f'모션 컴파일 실패: {error}')
                return None
            self._status(f'모션 {self.motion["time"][-1]:.2f}초, {len(self.motion["time"])}프레임 · '
                         f'최대 고정 오차 {self.motion["max_pin_error_mm"]:.2f} mm (기구학 결과)')
        return self.motion

    def _play(self):
        if self.playing:
            return
        motion = self._compiled()
        if motion is None:
            return
        self.select(None)
        self.playing = True
        threading.Thread(target=self._play_loop, args=(motion,), daemon=True).start()

    def _play_loop(self, motion):
        start = time.perf_counter()
        for index, q in enumerate(motion['qpos']):
            if not self.playing:
                break
            self.show(q)
            self.server.timeline.set_current_frame(index)
            delay = start + (index + 1) / FPS - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
        self.playing = False
        self.show(self.q)

    def _stop(self):
        self.playing = False


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', default='g1', choices=['g1', 'g1-tools'])
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--motions-dir', type=Path, default=DEFAULT_MOTIONS_DIR)
    args = parser.parse_args()
    server = viser.ViserServer(host=args.host, port=args.port, label='motion-sim')
    Editor(server, Robot(args.model), args.motions_dir)
    while True:
        time.sleep(3600)


if __name__ == '__main__':
    main()

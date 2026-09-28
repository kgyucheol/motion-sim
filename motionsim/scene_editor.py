"""Scene tab of the editor: place primitives and Isaac assets, edit their properties, show physics replays."""
import copy
import threading

import numpy as np
import viser

from .scene import (CATALOG, CATALOG_PATHS, PRIMITIVES, cached_asset_mesh, fixed_only, new_object, object_mesh,
                    rest_on_ground, store_asset_mesh, validate_objects)

ADD_LABELS = {'상자': 'box', '공': 'sphere', '원기둥': 'cylinder'}
NO_OBJECT = '(선택 없음)'


def hex_to_rgb(color):
    return tuple(int(color[i:i + 2], 16) for i in (1, 3, 5))


def rgb_to_hex(rgb):
    return '#' + ''.join(f'{int(c):02x}' for c in rgb)


def by_user(handler):
    """Wrap a GUI callback so it ignores programmatic value changes (viser reports them with client=None)."""
    def callback(event):
        if getattr(event, 'client', None) is not None:
            handler()
    return callback


class SceneEditor:
    def __init__(self, editor):
        self.editor, self.server = editor, editor.server
        self.selected = None
        self.nodes = {}
        self.gizmo = None
        self.drag_active = False
        self.catalog_names = {item['name']: item['path'] for item in CATALOG['assets']}

    @property
    def objects(self):
        return self.editor.project.setdefault('scene_objects', [])

    def _find(self, identifier):
        return next((item for item in self.objects if item['id'] == identifier), None)

    # -------------------------------------------------------------------- GUI
    def build_gui(self, gui):
        gui.add_markdown('물체를 배치하면 저장되고, `검증` 탭의 Isaac 물리에도 같은 장면이 들어갑니다. '
                         '물체를 클릭하거나 목록에서 고른 뒤 기즈모로 옮깁니다.')
        gui.add_button_group('도형 추가', list(ADD_LABELS)).on_click(lambda event: self.add(ADD_LABELS[event.target.value]))
        self.gui_asset = gui.add_dropdown('Isaac 에셋', list(self.catalog_names))
        gui.add_button('에셋 추가', icon=viser.Icon.PACKAGE).on_click(lambda _event: self.add_asset(self.catalog_names[self.gui_asset.value]))
        self.gui_list = gui.add_dropdown('물체', [NO_OBJECT])
        self.gui_list.on_update(by_user(self._on_list))
        self.gui_ground = gui.add_checkbox('옮긴 뒤 바닥에 내려놓기', True, hint='끄면 물체를 다른 물체 위나 공중에 둘 수 있습니다.')
        with gui.add_folder('선택 물체') as folder:
            self.gui_folder = folder
            self.gui_name = gui.add_text('이름', '')
            self.gui_name.on_update(by_user(self._on_fields))
            self.gui_size = gui.add_vector3('크기 (m)', (.3, .3, .3), min=(.01, .01, .01), max=(5, 5, 5), step=.01,
                                            hint='상자: 가로·세로·높이, 공: 지름, 원기둥: 지름·지름·높이')
            self.gui_size.on_update(by_user(self._on_fields))
            self.gui_scale = gui.add_number('배율', 1., min=.05, max=20., step=.05, hint='Isaac 에셋의 크기 배율')
            self.gui_scale.on_update(by_user(self._on_fields))
            self.gui_mass = gui.add_number('질량 (kg)', 1., min=0., max=500., step=.1, hint='에셋에서 0이면 에셋에 정의된 질량을 씁니다.')
            self.gui_mass.on_update(by_user(self._on_fields))
            self.gui_friction = gui.add_number('마찰 계수', .8, min=0., max=2., step=.05)
            self.gui_friction.on_update(by_user(self._on_fields))
            self.gui_color = gui.add_rgb('색', (200, 162, 107))
            self.gui_color.on_update(by_user(self._on_fields))
            self.gui_opacity = gui.add_slider('불투명도', min=.1, max=1., step=.05, initial_value=1.)
            self.gui_opacity.on_update(by_user(self._on_fields))
            self.gui_fixed = gui.add_checkbox('고정 (움직이지 않음)', False)
            self.gui_fixed.on_update(by_user(self._on_fields))
            gui.add_button('바닥에 내려놓기', icon=viser.Icon.ARROW_BAR_TO_DOWN).on_click(lambda _event: self.drop_selected())
            gui.add_button('복제', icon=viser.Icon.COPY).on_click(lambda _event: self.duplicate())
            gui.add_button('삭제', icon=viser.Icon.TRASH, color='red').on_click(lambda _event: self.delete())
            gui.add_button('선택 해제', icon=viser.Icon.X).on_click(lambda _event: self.select(None))
        self._sync_gui()

    def _sync_gui(self):
        options = [NO_OBJECT, *(f'{item["name"]}' for item in self.objects)]
        self.gui_list.options = options
        item = self._find(self.selected)
        self.gui_list.value = item['name'] if item else NO_OBJECT
        self.gui_folder.visible = item is not None
        if item is None:
            return
        primitive = item['kind'] in PRIMITIVES
        self.gui_size.visible = self.gui_friction.visible = primitive
        self.gui_scale.visible = not primitive
        self.gui_fixed.disabled = fixed_only(item)
        values = [(self.gui_name, item['name']), (self.gui_mass, float(item['mass_kg'] or 0.)),
                  (self.gui_color, hex_to_rgb(item['color'])), (self.gui_opacity, float(item.get('opacity', 1.))),
                  (self.gui_fixed, bool(item['fixed']))]
        if primitive:
            values += [(self.gui_size, tuple(item['size'])), (self.gui_friction, float(item['friction']))]
        else:
            values.append((self.gui_scale, float(item['scale'])))
        for handle, value in values:
            if handle.value != value:
                handle.value = value

    def _on_list(self):
        item = next((entry for entry in self.objects if entry['name'] == self.gui_list.value), None)
        self.select(item['id'] if item else None)

    def _on_fields(self):
        item = self._find(self.selected)
        if item is None or self.editor.playing:
            return
        before = copy.deepcopy(item)
        item['name'] = self.gui_name.value.strip()[:80] or before['name']
        item['color'] = rgb_to_hex(self.gui_color.value)
        item['opacity'] = float(self.gui_opacity.value)
        item['fixed'] = bool(self.gui_fixed.value) or fixed_only(item)
        if item['kind'] in PRIMITIVES:
            size = np.clip(np.asarray(self.gui_size.value, dtype=float), .01, 5.)
            if item['kind'] == 'sphere':
                changed = int(np.argmax(np.abs(size - np.asarray(before['size']))))
                size[:] = size[changed]
            elif item['kind'] == 'cylinder':
                size[1] = size[0] if size[0] != before['size'][0] else size[1]
                size[0] = size[1]
            item['size'] = size.tolist()
            item['friction'] = float(np.clip(self.gui_friction.value, 0., 2.))
            item['mass_kg'] = float(max(self.gui_mass.value, .001))
        else:
            item['scale'] = float(np.clip(self.gui_scale.value, .05, 20.))
            item['mass_kg'] = float(self.gui_mass.value) if self.gui_mass.value > 0 else None
        if item == before:
            return
        self.editor._checkpoint(coalesce=f'object:{item["id"]}')
        geometry_changed = item.get('size') != before.get('size') or item.get('scale') != before.get('scale')
        if geometry_changed and self.gui_ground.value:
            self._rest(item)
        self.editor._changed_scene()
        self._render(item)
        self._sync_gui()

    # ------------------------------------------------------------- objects
    def add(self, kind, asset=None):
        if self.editor.playing:
            return
        item = new_object(kind, self.objects, asset)
        item['position'] = self._free_spot(item)
        self.editor._checkpoint()
        self._rest(item)
        self.objects.append(item)
        validate_objects(self.objects)
        self.editor._changed_scene()
        self._render(item)
        self.select(item['id'])
        self.editor._status(f'{item["name"]}을(를) 추가했습니다.', 'ok')

    def add_asset(self, asset):
        if cached_asset_mesh(asset) is not None:
            self.add('asset', asset)
            return
        if self.editor.isaac.state not in ('ready', 'running'):
            self.editor._status('처음 쓰는 에셋은 Isaac 워커에서 모양을 받아야 합니다. `검증` 탭에서 워커를 시작한 뒤 다시 추가하세요.', 'warn')
            return
        self.editor._status('Isaac 워커에서 에셋 모양을 가져오는 중입니다.')

        def fetch():
            try:
                store_asset_mesh(asset, *self.editor.isaac.asset_mesh(asset))
            except RuntimeError as error:
                self.editor._status(str(error), 'error')
                return
            self.add('asset', asset)
        threading.Thread(target=fetch, daemon=True).start()

    def _free_spot(self, item):
        """In front of the robot, stepping sideways past objects that are already there."""
        for y in (0., -.45, .45, -.9, .9, -1.35, 1.35):
            if all(np.hypot(other['position'][0] - .7, other['position'][1] - y) > .4 for other in self.objects):
                return [.7, y, 0.]
        return [.7 + .1 * len(self.objects), 0., 0.]

    def _rest(self, item):
        mesh = object_mesh(item)
        if mesh is not None:
            rest_on_ground(item, mesh[0])

    def duplicate(self):
        item = self._find(self.selected)
        if item is None or self.editor.playing:
            return
        copy_item = new_object(item['kind'], self.objects, item.get('asset'))
        copy_item.update({key: copy.deepcopy(value) for key, value in item.items() if key not in ('id', 'name')})
        copy_item['position'] = [item['position'][0], item['position'][1] + .4, item['position'][2]]
        self.editor._checkpoint()
        self.objects.append(copy_item)
        self.editor._changed_scene()
        self._render(copy_item)
        self.select(copy_item['id'])

    def delete(self):
        item = self._find(self.selected)
        if item is None or self.editor.playing:
            return
        self.editor._checkpoint()
        self.objects.remove(item)
        self.select(None)
        self.editor._changed_scene()
        self.refresh()
        self.editor._status(f'{item["name"]}을(를) 삭제했습니다. 실행 취소로 되돌릴 수 있습니다.', 'ok')

    def drop_selected(self):
        item = self._find(self.selected)
        if item is None or self.editor.playing:
            return
        self.editor._checkpoint()
        self._rest(item)
        self.editor._changed_scene()
        self._render(item)
        self._place_gizmo(item)

    # -------------------------------------------------------------- display
    def refresh(self):
        """Recreate every object node (after undo, open, or new project)."""
        for node in self.nodes.values():
            node.remove()
        self.nodes = {}
        for item in self.objects:
            self._render(item)
        if self._find(self.selected) is None:
            self.select(None)
        self._sync_gui()

    def _render(self, item):
        mesh = object_mesh(item)
        if mesh is None:   # asset shape not fetched yet: show its catalog bounding box standing on the origin
            import trimesh
            box = trimesh.creation.box(extents=np.asarray(CATALOG_PATHS[item['asset']]['size_m']) * float(item['scale']))
            box.apply_translation([0, 0, box.extents[2] / 2])
            mesh = (box.vertices, box.faces)
            opacity = .35
            if self.editor.isaac.state == 'ready':
                threading.Thread(target=self._fetch_and_render, args=(item['asset'],), daemon=True).start()
        else:
            opacity = float(item.get('opacity', 1.))
        if item['id'] in self.nodes:
            self.nodes[item['id']].remove()
        node = self.server.scene.add_mesh_simple(
            f'/objects/{item["id"]}', mesh[0], mesh[1], color=hex_to_rgb(item['color']),
            opacity=None if opacity >= 1 else opacity, position=np.asarray(item['position']), wxyz=np.asarray(item['wxyz']))
        node.on_click(lambda _event, identifier=item['id']: self.select(identifier))
        self.nodes[item['id']] = node

    def _fetch_and_render(self, asset):
        try:
            store_asset_mesh(asset, *self.editor.isaac.asset_mesh(asset))
        except RuntimeError:
            return
        for item in self.objects:
            if item.get('asset') == asset:
                self._render(item)

    def show_poses(self, poses):
        """Move object nodes to physics poses {id: (position, wxyz)} during a replay."""
        for identifier, (position, wxyz) in poses.items():
            if identifier in self.nodes:
                self.nodes[identifier].position, self.nodes[identifier].wxyz = np.asarray(position), np.asarray(wxyz)

    def show_authored(self):
        for item in self.objects:
            if item['id'] in self.nodes:
                self.nodes[item['id']].position = np.asarray(item['position'])
                self.nodes[item['id']].wxyz = np.asarray(item['wxyz'])

    # ------------------------------------------------------------ selection
    def select(self, identifier):
        if self.editor.playing:
            return
        if identifier is not None and self.editor.selected is not None:
            self.editor.select(None)
        self.selected = identifier if self._find(identifier) else None
        if self.gizmo is not None:
            self.gizmo.remove()
            self.gizmo = None
        item = self._find(self.selected)
        if item is not None:
            self.gizmo = self.server.scene.add_transform_controls('/object_gizmo', scale=.3, depth_test=False,
                                                                 position=np.asarray(item['position']),
                                                                 wxyz=np.asarray(item['wxyz']))
            self.gizmo.on_drag_start(lambda _event: self._on_drag_start())
            self.gizmo.on_update(lambda _event: self._on_drag_update())
            self.gizmo.on_drag_end(lambda _event: self._on_drag_end())
        self._sync_gui()
        self.editor._render_status()

    def _place_gizmo(self, item):
        if self.gizmo is not None:
            self.gizmo.position, self.gizmo.wxyz = np.asarray(item['position']), np.asarray(item['wxyz'])

    def _on_drag_start(self):
        if self.editor.playing:
            return
        self.editor._checkpoint()
        self.drag_active = True

    def _on_drag_update(self):
        item = self._find(self.selected)
        if item is None or not self.drag_active:
            return
        item['position'] = np.asarray(self.gizmo.position, dtype=float).tolist()
        wxyz = np.asarray(self.gizmo.wxyz, dtype=float)
        item['wxyz'] = (wxyz / np.linalg.norm(wxyz)).tolist()
        node = self.nodes.get(item['id'])
        if node is not None:
            node.position, node.wxyz = np.asarray(item['position']), np.asarray(item['wxyz'])

    def _on_drag_end(self):
        self.drag_active = False
        item = self._find(self.selected)
        if item is None:
            return
        if self.gui_ground.value:
            self._rest(item)
            self._render(item)
            self._place_gizmo(item)
        self.editor._changed_scene()

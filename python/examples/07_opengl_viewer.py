#!/usr/bin/env python3
"""OpenGL viewer example for nanoprc_py.

This example loads a PRC/PDF using the nanoprc_py bindings and renders
face geometry with VBO/EBO upload, per-face diffuse material, camera
orbit controls, and tessellation selection.
"""

import math
import sys
from dataclasses import dataclass

import glfw
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import gluPerspective

from nanoprc_py import Context

PRIMITIVE_MODE_MAP = {
    0: GL_TRIANGLES,
    1: GL_TRIANGLE_FAN,
    2: GL_TRIANGLE_STRIP,
    3: GL_LINES,
    4: GL_LINE_STRIP,
    5: GL_LINE_LOOP,
}

DEFAULT_DIFFUSE = (0.85, 0.85, 0.9, 1.0)
WIREFRAME_COLOR = (0.15, 0.8, 0.95, 1.0)


@dataclass
class Mesh:
    vbo: int
    ebo: int
    nbo: int
    index_count: int
    mode: int
    color: tuple[float, float, float, float]
    face_index: int
    primitive_index: int
    has_normals: bool


class NanoPRCViewer:
    def __init__(self, filename: str) -> None:
        self.filename = filename
        self.window = None
        self.width = 1280
        self.height = 800
        self.mouse_down = False
        self.last_cursor = (0.0, 0.0)
        self.yaw = 35.0
        self.pitch = -20.0
        self.distance = 6.0
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.draw_wireframe = False
        self.use_lighting = True
        self.use_material = True
        self.current_tess = 0
        self.show_all_tess = False
        self.show_attribute_window = True
        self.tess_count = 0
        self.meshes: list[Mesh] = []
        self.document = None
        self.model_root = None
        self.attribute_records: list[dict] = []
        self.scene_center = np.zeros(3, dtype=np.float32)
        self.scene_radius = 1.0
        self.min_distance = 0.5
        self.max_distance = 50.0
        self.zoom_step = 0.2
        self.pan_step = 0.1
        self.light_position = np.array([2.0, 2.0, 2.0, 1.0], dtype=np.float32)
        self.light_step = 0.25
        self.light_intensity = 1.0
        self.specular_intensity = 1.2
        self.shininess = 64.0
        self.invert_normals = False
        self._tk = None
        self._tk_root = None
        self._attr_window = None
        self._attr_text = None
        self._attr_label = None
        self._last_attr_text = ""

    def run(self) -> None:
        self._init_glfw()
        self._load_document()
        self._build_meshes()
        self._main_loop()
        self._cleanup()

    def _init_glfw(self) -> None:
        if not glfw.init():
            raise RuntimeError("Failed to initialize GLFW")

        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 2)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 1)
        glfw.window_hint(glfw.RESIZABLE, True)
        glfw.window_hint(glfw.SAMPLES, 4)

        self.window = glfw.create_window(self.width, self.height, "nanoPRC OpenGL Viewer", None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("Failed to create GLFW window")

        glfw.make_context_current(self.window)
        glfw.set_framebuffer_size_callback(self.window, self._on_resize)
        glfw.set_key_callback(self.window, self._on_key)
        glfw.set_mouse_button_callback(self.window, self._on_mouse_button)
        glfw.set_cursor_pos_callback(self.window, self._on_cursor_move)
        glfw.set_scroll_callback(self.window, self._on_scroll)

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_CULL_FACE)
        glCullFace(GL_BACK)
        glEnable(GL_MULTISAMPLE)
        glClearColor(0.08, 0.08, 0.12, 1.0)

    def _load_document(self) -> None:
        ctx = Context()
        self.document = ctx.open(self.filename)
        self.document.prepare_model_tree()
        self.model_root = self.document.create_model_tree()
        tess_count, _ = self.document.tessellation_counts()
        self.tess_count = int(tess_count)
        self._build_attribute_records()
        self._init_attribute_window()
        if self.tess_count == 0:
            raise RuntimeError("No surface tessellations found in document")
        print(f"Loaded {self.filename}")
        print(f"Surface tessellation count: {self.tess_count}")
        print("Controls:")
        print("  Left drag = rotate")
        print("  Scroll / W/S = zoom")
        print("  Arrow keys = pan")
        print("  T = cycle tessellation")
        print("  A = toggle all tessellations")
        print("  P = toggle attributes window")
        print("  R = refit view")
        print("  L = toggle lighting")
        print("  N = invert normals (diagnostic)")
        print("  I/K H/; U/O = move light (+Y/-Y, -X/+X, -Z/+Z)")
        print("  [ / ] = decrease/increase light intensity")
        print("  , / . = decrease/increase specular intensity")
        print("  M = toggle material colors")
        print("  F = toggle wireframe overlay")
        print("  ESC = exit")

    def _build_attribute_records(self) -> None:
        self.attribute_records = []

        def walk(node, index_counter):
            idx = index_counter[0]
            index_counter[0] += 1
            self.attribute_records.append({
                "index": idx,
                "node_name": node.name or "",
                "part_name": node.part_name if node.has_part else "",
                "node_attributes": node.attributes(),
                "part_attributes": node.part_attributes(),
            })
            for child in node.children():
                walk(child, index_counter)

        if self.model_root is not None:
            walk(self.model_root, [0])

    def _init_attribute_window(self) -> None:
        try:
            import tkinter as tk
            from tkinter import ttk
        except Exception:
            self.show_attribute_window = False
            return

        self._tk = tk
        self._tk_root = tk.Tk()
        self._tk_root.withdraw()
        self._attr_window = tk.Toplevel(self._tk_root)
        self._attr_window.title("Tessellation Attributes")
        self._attr_window.geometry("560x420")

        frame = ttk.Frame(self._attr_window)
        frame.pack(fill=tk.BOTH, expand=True)

        self._attr_label = ttk.Label(frame, text="", anchor="w")
        self._attr_label.pack(fill=tk.X, padx=6, pady=(6, 2))

        text_frame = ttk.Frame(frame)
        text_frame.pack(fill=tk.BOTH, expand=True, padx=6, pady=(0, 6))

        scrollbar = ttk.Scrollbar(text_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self._attr_text = tk.Text(text_frame, wrap=tk.WORD, yscrollcommand=scrollbar.set)
        self._attr_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self._attr_text.yview)

        def on_close():
            self.show_attribute_window = False
            self._attr_window.withdraw()

        self._attr_window.protocol("WM_DELETE_WINDOW", on_close)

    def _format_attribute_bases(self, bases: list[dict], indent: str = "") -> str:
        lines: list[str] = []
        for base in bases:
            base_title = base.get("base_title") or "(no base title)"
            lines.append(f"{indent}Base: {base_title}")
            for entry in base.get("entries", []):
                entry_title = entry.get("entry_title") or "(untitled)"
                lines.append(
                    f"{indent}  {entry_title} = {entry.get('value')!r} (type={entry.get('type')})")
        return "\n".join(lines)

    def _attributes_text_for_current_tess(self):
        if self.show_all_tess:
            return ("All-tessellations mode enabled.\n"
                "Press A to return to single-tessellation mode for per-tess attributes.")

        info = self.document.tessellation_info(self.current_tess)
        tess_name = info.get("name") or ""
        product_index = int(info.get("product_index", -1))

        record = None
        if 0 <= product_index < len(self.attribute_records):
            record = self.attribute_records[product_index]

        if record is None and tess_name:
            for rec in self.attribute_records:
                if rec["part_name"] == tess_name:
                    record = rec
                    break
            if record is None:
                for rec in self.attribute_records:
                    if rec["node_name"] == tess_name:
                        record = rec
                        break

        lines = [
            f"Tessellation index: {self.current_tess}",
            f"Name: {tess_name if tess_name else '(unnamed)'}",
            f"Product index: {product_index}",
            "",
        ]

        if record is None:
            lines.append("No matching model-tree node was resolved for this tessellation.")
            return "\n".join(lines)

        node_name = record.get("node_name") or "(unnamed node)"
        part_name = record.get("part_name") or "(no part)"
        lines.append(f"Matched node: {node_name}")
        lines.append(f"Matched part: {part_name}")

        node_attrs = record.get("node_attributes") or []
        part_attrs = record.get("part_attributes") or []

        if node_attrs:
            lines.append("")
            lines.append("Node attributes:")
            lines.append(self._format_attribute_bases(node_attrs, "  "))

        if part_attrs:
            lines.append("")
            lines.append("Part attributes:")
            lines.append(self._format_attribute_bases(part_attrs, "  "))

        if not node_attrs and not part_attrs:
            lines.append("")
            lines.append("No attributes on matched node/part.")

        return "\n".join(lines)

    def _update_attribute_window(self) -> None:
        if self._attr_window is None or self._attr_text is None:
            return

        try:
            state = self._attr_window.state()
        except Exception:
            return

        if not self.show_attribute_window:
            if state != "withdrawn":
                self._attr_window.withdraw()
            return

        if state == "withdrawn":
            # Only show once when toggled on; repeatedly deiconifying each frame
            # can re-activate the Tk window and steal keyboard focus from GLFW.
            self._attr_window.deiconify()

        if self._attr_label is not None:
            mode = "all" if self.show_all_tess else f"{self.current_tess + 1}/{self.tess_count}"
            self._attr_label.config(text=f"Attributes (tess={mode})")

        text = self._attributes_text_for_current_tess()
        if text != self._last_attr_text:
            self._attr_text.delete("1.0", self._tk.END)
            self._attr_text.insert(self._tk.END, text)
            self._last_attr_text = text

        if self._tk_root is not None:
            self._tk_root.update_idletasks()
            self._tk_root.update()

    def _build_meshes(self) -> None:
        self._destroy_meshes()
        self.meshes = []
        bounds_min = np.array([np.inf, np.inf, np.inf], dtype=np.float64)
        bounds_max = np.array([-np.inf, -np.inf, -np.inf], dtype=np.float64)
        if self.show_all_tess:
            tess_indices = range(self.tess_count)
            print("Building meshes for all tessellations")
        else:
            tess_indices = [self.current_tess]
            print(f"Building meshes for tessellation {self.current_tess}")

        for tess_index in tess_indices:
            num_faces = int(self.document.number_of_faces(tess_index))
            for face_index in range(num_faces):
                positions = np.asarray(self.document.face_vertex_positions(tess_index, face_index), dtype=np.float32)
                if positions.size == 0:
                    continue

                face_positions = positions.reshape((-1, 3))
                bounds_min = np.minimum(bounds_min, face_positions.min(axis=0))
                bounds_max = np.maximum(bounds_max, face_positions.max(axis=0))

                primitive_count = int(self.document.face_graphics_primitive_count(tess_index, face_index))
                if primitive_count == 0:
                    continue

                material = self.document.face_material(tess_index, face_index)
                diffuse = self._extract_diffuse_color(material)
                color = diffuse if self.use_material else DEFAULT_DIFFUSE

                for primitive_index in range(primitive_count):
                    primitive = self.document.get_graphics_primitive(tess_index, face_index, primitive_index)
                    indices = np.asarray(primitive["indices"], dtype=np.uint32)
                    if indices.size == 0:
                        continue

                    mode = PRIMITIVE_MODE_MAP.get(int(primitive["type"]), GL_TRIANGLES)
                    normals = self._get_face_normals(tess_index, face_index)
                    if normals is None or normals.shape[0] != face_positions.shape[0]:
                        normals = self._build_vertex_normals(face_positions, indices, mode)
                    has_normals = normals is not None
                    vbo = glGenBuffers(1)
                    ebo = glGenBuffers(1)
                    nbo = glGenBuffers(1) if has_normals else 0

                    glBindBuffer(GL_ARRAY_BUFFER, vbo)
                    glBufferData(GL_ARRAY_BUFFER, positions.nbytes, positions, GL_STATIC_DRAW)

                    if has_normals:
                        glBindBuffer(GL_ARRAY_BUFFER, nbo)
                        glBufferData(GL_ARRAY_BUFFER, normals.nbytes, normals, GL_STATIC_DRAW)

                    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo)
                    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, GL_STATIC_DRAW)

                    self.meshes.append(Mesh(
                        vbo=vbo,
                        ebo=ebo,
                        nbo=nbo,
                        index_count=indices.size,
                        mode=mode,
                        color=color,
                        face_index=face_index,
                        primitive_index=primitive_index,
                        has_normals=has_normals,
                    ))

        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)
        self._update_view_from_bounds(bounds_min, bounds_max)
        print(f"Created {len(self.meshes)} draw primitives")

    def _build_vertex_normals(self, positions: np.ndarray, indices: np.ndarray, mode: int):
        if mode not in (GL_TRIANGLES, GL_TRIANGLE_STRIP, GL_TRIANGLE_FAN):
            return None

        nverts = positions.shape[0]
        normals = np.zeros((nverts, 3), dtype=np.float32)

        def add_tri(i0: int, i1: int, i2: int) -> None:
            if i0 >= nverts or i1 >= nverts or i2 >= nverts:
                return
            p0 = positions[i0]
            p1 = positions[i1]
            p2 = positions[i2]
            n = np.cross(p1 - p0, p2 - p0)
            length = np.linalg.norm(n)
            if length < 1e-12:
                return
            normals[i0] += n
            normals[i1] += n
            normals[i2] += n

        if mode == GL_TRIANGLES:
            for i in range(0, int(indices.size) - 2, 3):
                add_tri(int(indices[i]), int(indices[i + 1]), int(indices[i + 2]))
        elif mode == GL_TRIANGLE_STRIP:
            for i in range(0, int(indices.size) - 2):
                a = int(indices[i])
                b = int(indices[i + 1])
                c = int(indices[i + 2])
                if i % 2 == 1:
                    a, b = b, a
                add_tri(a, b, c)
        else:  # GL_TRIANGLE_FAN
            base = int(indices[0])
            for i in range(1, int(indices.size) - 1):
                add_tri(base, int(indices[i]), int(indices[i + 1]))

        lengths = np.linalg.norm(normals, axis=1)
        valid = lengths > 1e-12
        if np.any(valid):
            normals[valid] = normals[valid] / lengths[valid, np.newaxis]
        if np.any(~valid):
            normals[~valid] = np.array([0.0, 0.0, 1.0], dtype=np.float32)

        if self.invert_normals:
            normals = -normals

        return np.ascontiguousarray(normals, dtype=np.float32)

    def _get_face_normals(self, tess_index: int, face_index: int):
        normals = np.asarray(self.document.face_vertex_normals(tess_index, face_index), dtype=np.float32)
        if normals.size == 0:
            return None

        normals = normals.reshape((-1, 3))
        lengths = np.linalg.norm(normals, axis=1)
        valid = lengths > 1e-12
        if not np.any(valid):
            return None

        out = np.zeros_like(normals, dtype=np.float32)
        out[valid] = normals[valid] / lengths[valid, np.newaxis]
        if np.any(~valid):
            out[~valid] = np.array([0.0, 0.0, 1.0], dtype=np.float32)

        if self.invert_normals:
            out = -out

        return np.ascontiguousarray(out, dtype=np.float32)

    def _update_view_from_bounds(self, bounds_min: np.ndarray, bounds_max: np.ndarray) -> None:
        if not np.all(np.isfinite(bounds_min)) or not np.all(np.isfinite(bounds_max)):
            self.scene_center = np.zeros(3, dtype=np.float32)
            self.scene_radius = 1.0
        else:
            center = (bounds_min + bounds_max) * 0.5
            extent = bounds_max - bounds_min
            radius = float(np.linalg.norm(extent) * 0.5)
            self.scene_center = center.astype(np.float32)
            self.scene_radius = max(radius, 0.1)

        self.zoom_step = max(self.scene_radius * 0.10, 0.05)
        self.pan_step = max(self.scene_radius * 0.05, 0.02)
        self.min_distance = max(self.scene_radius * 0.05, 0.05)
        self.max_distance = max(self.scene_radius * 40.0, self.min_distance + 5.0)
        self.distance = max(self.min_distance, min(self.max_distance, self.scene_radius * 2.5))
        self.light_step = max(self.scene_radius * 0.10, 0.05)
        self.light_position = np.array([
            self.scene_center[0] + self.scene_radius * 1.8,
            self.scene_center[1] + self.scene_radius * 1.8,
            self.scene_center[2] + self.scene_radius * 1.8,
            1.0,
        ], dtype=np.float32)
        self.offset_x = 0.0
        self.offset_y = 0.0

    def _clamp_distance(self) -> None:
        self.distance = max(self.min_distance, min(self.max_distance, self.distance))

    def _refit_view(self) -> None:
        self.distance = max(self.min_distance, min(self.max_distance, self.scene_radius * 2.5))
        self.offset_x = 0.0
        self.offset_y = 0.0

    def _destroy_meshes(self) -> None:
        for mesh in self.meshes:
            glDeleteBuffers(1, [mesh.vbo])
            glDeleteBuffers(1, [mesh.ebo])
            if mesh.nbo:
                glDeleteBuffers(1, [mesh.nbo])
        self.meshes = []

    def _extract_diffuse_color(self, material: dict) -> tuple[float, float, float, float]:
        if not material or not material.get("is_material", False):
            return DEFAULT_DIFFUSE

        mat = material.get("material", {})
        diffuse = mat.get("diffuse")
        alpha = float(mat.get("diffuse_alpha", 1.0))
        if diffuse is None or len(diffuse) < 3:
            return DEFAULT_DIFFUSE

        try:
            r, g, b = float(diffuse[0]), float(diffuse[1]), float(diffuse[2])
            if alpha == 0.0:
                alpha = 1.0
            return (r, g, b, alpha)
        except Exception:
            return DEFAULT_DIFFUSE

    def _on_resize(self, window, width: int, height: int) -> None:
        self.width = width
        self.height = max(1, height)
        glViewport(0, 0, self.width, self.height)

    def _on_key(self, window, key, scancode, action, mods) -> None:
        if action != glfw.PRESS:
            return

        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(window, True)
        elif key == glfw.KEY_T:
            self.current_tess = (self.current_tess + 1) % self.tess_count
            if not self.show_all_tess:
                self._build_meshes()
        elif key == glfw.KEY_A:
            self.show_all_tess = not self.show_all_tess
            self._build_meshes()
        elif key == glfw.KEY_P:
            self.show_attribute_window = not self.show_attribute_window
        elif key == glfw.KEY_R:
            self._refit_view()
        elif key == glfw.KEY_M:
            self.use_material = not self.use_material
            self._build_meshes()
        elif key == glfw.KEY_L:
            self.use_lighting = not self.use_lighting
        elif key == glfw.KEY_N:
            self.invert_normals = not self.invert_normals
            self._build_meshes()
        elif key == glfw.KEY_F:
            self.draw_wireframe = not self.draw_wireframe
        elif key == glfw.KEY_W:
            self.distance -= self.zoom_step
            self._clamp_distance()
        elif key == glfw.KEY_S:
            self.distance += self.zoom_step
            self._clamp_distance()
        elif key == glfw.KEY_UP:
            self.offset_y += self.pan_step
        elif key == glfw.KEY_DOWN:
            self.offset_y -= self.pan_step
        elif key == glfw.KEY_LEFT:
            self.offset_x -= self.pan_step
        elif key == glfw.KEY_RIGHT:
            self.offset_x += self.pan_step
        elif key == glfw.KEY_I:
            self.light_position[1] += self.light_step
        elif key == glfw.KEY_K:
            self.light_position[1] -= self.light_step
        elif key == glfw.KEY_H:
            self.light_position[0] -= self.light_step
        elif key == glfw.KEY_SEMICOLON:
            self.light_position[0] += self.light_step
        elif key == glfw.KEY_U:
            self.light_position[2] -= self.light_step
        elif key == glfw.KEY_O:
            self.light_position[2] += self.light_step
        elif key == glfw.KEY_LEFT_BRACKET:
            self.light_intensity = max(0.05, self.light_intensity * 0.9)
        elif key == glfw.KEY_RIGHT_BRACKET:
            self.light_intensity = min(5.0, self.light_intensity * 1.1)
        elif key == glfw.KEY_COMMA:
            self.specular_intensity = max(0.0, self.specular_intensity * 0.85)
        elif key == glfw.KEY_PERIOD:
            self.specular_intensity = min(6.0, self.specular_intensity * 1.15)

    def _on_mouse_button(self, window, button, action, mods) -> None:
        if button == glfw.MOUSE_BUTTON_LEFT:
            self.mouse_down = action == glfw.PRESS

    def _on_cursor_move(self, window, xpos: float, ypos: float) -> None:
        if not self.mouse_down:
            self.last_cursor = (xpos, ypos)
            return

        dx = xpos - self.last_cursor[0]
        dy = ypos - self.last_cursor[1]
        self.last_cursor = (xpos, ypos)

        self.yaw += float(dx) * 0.35
        self.pitch += float(dy) * 0.35
        self.pitch = max(-89.9, min(89.9, self.pitch))

    def _on_scroll(self, window, xoffset: float, yoffset: float) -> None:
        self.distance *= math.pow(0.92, yoffset)
        self._clamp_distance()

    def _setup_camera(self) -> None:
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        # Keep depth range tight around the model to reduce z-fighting artifacts,
        # especially when all tessellations are rendered and the camera is far away.
        padding = max(self.scene_radius * 0.25, 0.1)
        near = max(0.01, self.distance - self.scene_radius - padding)
        far = self.distance + self.scene_radius + padding
        if far <= near + 1.0:
            far = near + 1.0
        gluPerspective(45.0, self.width / self.height, near, far)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(self.offset_x, self.offset_y, -self.distance)

    def _apply_scene_transform(self) -> None:
        glRotatef(self.pitch, 1.0, 0.0, 0.0)
        glRotatef(self.yaw, 0.0, 1.0, 0.0)
        glTranslatef(-float(self.scene_center[0]), -float(self.scene_center[1]),
            -float(self.scene_center[2]))

    def _setup_lighting(self) -> None:
        if not self.use_lighting:
            glDisable(GL_LIGHTING)
            glDisable(GL_LIGHT0)
            return

        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_NORMALIZE)
        glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE)
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE)

        ambient = 0.12 * self.light_intensity
        diffuse = 0.70 * self.light_intensity
        specular = self.specular_intensity * self.light_intensity

        glLightfv(GL_LIGHT0, GL_POSITION, self.light_position)
        glLightfv(GL_LIGHT0, GL_AMBIENT, (GLfloat * 4)(ambient, ambient, ambient, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (GLfloat * 4)(diffuse, diffuse, diffuse, 1.0))
        glLightfv(GL_LIGHT0, GL_SPECULAR, (GLfloat * 4)(specular, specular, specular, 1.0))

        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, (GLfloat * 4)(1.0, 1.0, 1.0, 1.0))
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, self.shininess)

    def _draw_status(self) -> None:
        if self.show_all_tess:
            tess_label = "all"
        else:
            tess_label = f"{self.current_tess + 1}/{self.tess_count}"

        title = (
            f"nanoPRC Viewer - tess {tess_label} "
            f"| dist={self.distance:.2f} "
            f"| light={'on' if self.use_lighting else 'off'} "
            f"| lightpos=({self.light_position[0]:.2f},{self.light_position[1]:.2f},{self.light_position[2]:.2f}) "
            f"| spec={self.specular_intensity:.2f} "
            f"| normals={'inv' if self.invert_normals else 'std'} "
            f"| material={'on' if self.use_material else 'off'} "
            f"| wireframe={'on' if self.draw_wireframe else 'off'}"
        )
        glfw.set_window_title(self.window, title)

    def _render(self) -> None:
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self._setup_camera()
        self._setup_lighting()
        self._apply_scene_transform()

        glEnableClientState(GL_VERTEX_ARRAY)
        if self.use_lighting:
            glEnableClientState(GL_NORMAL_ARRAY)
        else:
            glDisableClientState(GL_NORMAL_ARRAY)
        glBindBuffer(GL_ARRAY_BUFFER, 0)

        for mesh in self.meshes:
            glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo)
            glVertexPointer(3, GL_FLOAT, 0, None)
            if self.use_lighting and mesh.has_normals:
                glBindBuffer(GL_ARRAY_BUFFER, mesh.nbo)
                glNormalPointer(GL_FLOAT, 0, None)
            elif self.use_lighting:
                glNormal3f(0.0, 0.0, 1.0)
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo)

            glColor4f(*mesh.color)
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
            glDrawElements(mesh.mode, mesh.index_count, GL_UNSIGNED_INT, None)

            if self.draw_wireframe:
                if self.use_lighting:
                    glDisable(GL_LIGHTING)
                glColor4f(*WIREFRAME_COLOR)
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
                glLineWidth(1.0)
                glDrawElements(mesh.mode, mesh.index_count, GL_UNSIGNED_INT, None)
                if self.use_lighting:
                    glEnable(GL_LIGHTING)

        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_NORMAL_ARRAY)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)

    def _main_loop(self) -> None:
        while not glfw.window_should_close(self.window):
            self._draw_status()
            self._render()
            self._update_attribute_window()
            glfw.swap_buffers(self.window)
            glfw.poll_events()

    def _cleanup(self) -> None:
        self._destroy_meshes()
        if self._attr_window is not None:
            try:
                self._attr_window.destroy()
            except Exception:
                pass
        if self._tk_root is not None:
            try:
                self._tk_root.destroy()
            except Exception:
                pass
        if self.window:
            glfw.destroy_window(self.window)
        glfw.terminate()


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    if len(argv) != 1:
        print("Usage: python 07_opengl_viewer.py path/to/file.prc")
        return 1

    viewer = NanoPRCViewer(argv[0])
    viewer.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

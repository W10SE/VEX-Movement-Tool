import tkinter as tk

from config import CANVAS_PX, FIELD_PX, FIELD_SIZE_M, MARGIN_PX, POINT_HIT_PX, SEGMENT_HIT_PX, TILE_SIZE_M
from core.geometry import point_segment_distance
from utils.converters import m_to_px, px_to_m


class PathCanvas(tk.Canvas):
    def __init__(self, master, state, on_change, **kwargs):
        super().__init__(master, width=CANVAS_PX, height=CANVAS_PX, bg="#1e1e1e", highlightthickness=0, **kwargs)
        self.state = state
        self.on_change = on_change
        self.dragging_index = None

        self.bind("<Button-1>", self._on_left_click)
        self.bind("<B1-Motion>", self._on_left_drag)
        self.bind("<ButtonRelease-1>", self._on_left_release)
        self.bind("<Button-3>", self._on_right_click)

        self._draw_static_field()

    def redraw_path(self):
        self.delete("path")

        if len(self.state.waypoints_m) >= 2:
            coords = []
            for x, y in self.state.waypoints_m:
                coords.extend(m_to_px(x, y))
            self.create_line(*coords, fill="#888888", width=1, dash=(3, 3), tags="path")

        if len(self.state.smooth_m) >= 2:
            coords = []
            for x, y in self.state.smooth_m:
                coords.extend(m_to_px(x, y))
            self.create_line(*coords, fill="#ffb300", width=3, smooth=False, tags="path")

        for x, y in self.state.simplified_m:
            px, py = m_to_px(x, y)
            self.create_oval(px - 5, py - 5, px + 5, py + 5, fill="#42a5f5", outline="white", tags="path")

        for i, (x, y) in enumerate(self.state.waypoints_m):
            px, py = m_to_px(x, y)
            is_first = i == 0
            is_last = i == len(self.state.waypoints_m) - 1
            color = "#00e676" if is_first else ("#ff5252" if is_last else "#ffffff")
            self.create_oval(px - 4, py - 4, px + 4, py + 4, fill=color, outline="black", tags="path")

    def _nearest_point_index(self, px, py):
        best_i, best_d = None, POINT_HIT_PX
        for i, (x, y) in enumerate(self.state.waypoints_m):
            qx, qy = m_to_px(x, y)
            d = ((qx - px) ** 2 + (qy - py) ** 2) ** 0.5
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def _nearest_segment_index(self, px, py):
        if len(self.state.waypoints_m) < 2:
            return None

        best_i, best_d = None, SEGMENT_HIT_PX
        for i in range(len(self.state.waypoints_m) - 1):
            a = m_to_px(*self.state.waypoints_m[i])
            b = m_to_px(*self.state.waypoints_m[i + 1])
            d = point_segment_distance((px, py), a, b)
            if d < best_d:
                best_d, best_i = d, i

        return best_i

    def _clamp_to_field(self, x_m, y_m):
        half = FIELD_SIZE_M / 2
        return max(-half, min(half, x_m)), max(-half, min(half, y_m))

    def _on_left_click(self, event):
        idx = self._nearest_point_index(event.x, event.y)
        if idx is not None:
            self.dragging_index = idx
            return

        seg_idx = self._nearest_segment_index(event.x, event.y)
        point = self._clamp_to_field(*px_to_m(event.x, event.y))

        if seg_idx is not None:
            self.state.add_point(point, index=seg_idx + 1)
        else:
            self.state.add_point(point)

        self.on_change()

    def _on_left_drag(self, event):
        if self.dragging_index is None:
            return
        point = self._clamp_to_field(*px_to_m(event.x, event.y))
        self.state.move_point(self.dragging_index, point)
        self.on_change()

    def _on_left_release(self, event):
        self.dragging_index = None

    def _on_right_click(self, event):
        idx = self._nearest_point_index(event.x, event.y)
        if idx is not None:
            self.state.remove_point(idx)
            self.on_change()

    def _draw_static_field(self):
        n_tiles = int(round(FIELD_SIZE_M / TILE_SIZE_M))
        colors = ["#8e8e8e", "#7f7f7f"]

        for row in range(n_tiles):
            for col in range(n_tiles):
                x0 = MARGIN_PX + col * (FIELD_PX / n_tiles)
                y0 = MARGIN_PX + row * (FIELD_PX / n_tiles)
                x1 = x0 + FIELD_PX / n_tiles
                y1 = y0 + FIELD_PX / n_tiles
                color = colors[(row + col) % 2]
                self.create_rectangle(x0, y0, x1, y1, fill=color, outline="#5a5a5a", width=1)

        self.create_rectangle(
            MARGIN_PX, MARGIN_PX, MARGIN_PX + FIELD_PX, MARGIN_PX + FIELD_PX,
            outline="#cfcfcf", width=3
        )

        x0, _ = m_to_px(-FIELD_SIZE_M / 2 - 0.15, 0)
        x1, _ = m_to_px(FIELD_SIZE_M / 2 + 0.15, 0)
        _, y0 = m_to_px(0, FIELD_SIZE_M / 2 + 0.15)
        _, y1 = m_to_px(0, -FIELD_SIZE_M / 2 - 0.15)
        cx, cy = m_to_px(0, 0)

        self.create_line(x0, cy, x1, cy, fill="#e53935", width=3, arrow=tk.BOTH)
        self.create_line(cx, y0, cx, y1, fill="#3f6bff", width=3, arrow=tk.BOTH)

        self.create_text(x1 + 14, cy, text="+X", fill="#e53935", font=("Segoe UI", 11, "bold"))
        self.create_text(x0 - 14, cy, text="-X", fill="#e53935", font=("Segoe UI", 11, "bold"))
        self.create_text(cx, y0 - 12, text="+Y", fill="#3f6bff", font=("Segoe UI", 11, "bold"))
        self.create_text(cx, y1 + 12, text="-Y", fill="#3f6bff", font=("Segoe UI", 11, "bold"))

        steps = int(round((FIELD_SIZE_M / 2) / TILE_SIZE_M))
        for i in range(-steps, steps + 1):
            if i == 0:
                continue

            v = round(i * TILE_SIZE_M, 2)
            px, py = m_to_px(v, 0)
            self.create_line(px, cy - 6, px, cy + 6, fill="#e53935", width=2)
            self.create_text(px, cy + 18, text=f"{v} m", fill="white", font=("Segoe UI", 9))

            px2, py2 = m_to_px(0, v)
            self.create_line(cx - 6, py2, cx + 6, py2, fill="#3f6bff", width=2)
            self.create_text(cx + 34, py2, text=f"{v} m", fill="white", font=("Segoe UI", 9))

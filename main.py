"""
VEX Autonomous Path Planner
============================
Draw a robot path on a to-scale VEX field, simplify/smooth it so it isn't
jittery, and export ready-to-use autonomous movement code for:

  1. LemLib (PROS C++)                - moveToPoint / moveToPose calls
  2. LemLib Pure-Pursuit path file     - the "x, y, speed ... endData" asset format
  3. VEXcode V5 C++ (turn + drive)     - Drivetrain.turnToHeading / driveFor
  4. VEXcode V5 Python (turn + drive)  - drivetrain.turn_to_heading / drive_for

Requirements:
    pip install customtkinter

Run:
    python vex_path_planner.py

Controls on the field canvas:
    Left click on empty space   -> add a new waypoint at the end of the path
    Left click near a path line -> insert a waypoint at that spot
    Left click + drag a point   -> move it
    Right click a point         -> delete it
    
"""

import math
import json
import tkinter as tk
from tkinter import filedialog, messagebox

import customtkinter as ctk

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# ---------------------------------------------------------------------------
# Field geometry (matches the provided field reference image)
# ---------------------------------------------------------------------------
FIELD_SIZE_M = 3.6      # field spans -1.8 m .. +1.8 m on both axes
TILE_SIZE_M = 0.6       # 6x6 tile grid
CANVAS_PX = 700
MARGIN_PX = 46
FIELD_PX = CANVAS_PX - 2 * MARGIN_PX
SCALE = FIELD_PX / FIELD_SIZE_M   # pixels per meter

UNIT_FACTORS = {"in": 39.37007874, "mm": 1000.0, "cm": 100.0, "m": 1.0}

POINT_HIT_PX = 10
SEGMENT_HIT_PX = 12


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def m_to_px(x_m, y_m):
    px = CANVAS_PX / 2 + x_m * SCALE
    py = CANVAS_PX / 2 - y_m * SCALE
    return px, py


def px_to_m(px, py):
    x_m = (px - CANVAS_PX / 2) / SCALE
    y_m = -(py - CANVAS_PX / 2) / SCALE
    return x_m, y_m


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_segment_distance(p, a, b):
    """Distance from point p to segment a-b, all in the same units."""
    if a == b:
        return dist(p, a)
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def heading_deg(dx, dy):
    """VEX-style heading: 0 = +Y (north), clockwise positive."""
    return math.degrees(math.atan2(dx, dy)) % 360.0


# ---------------------------------------------------------------------------
# Path simplification (Ramer-Douglas-Peucker) + smoothing (Catmull-Rom) (Made using AI)
# ---------------------------------------------------------------------------
def rdp(points, epsilon):
    if len(points) < 3:
        return points[:]
    start, end = points[0], points[-1]
    dmax, index = 0.0, 0
    for i in range(1, len(points) - 1):
        d = point_segment_distance(points[i], start, end)
        if d > dmax:
            dmax, index = d, i
    if dmax > epsilon:
        left = rdp(points[: index + 1], epsilon)
        right = rdp(points[index:], epsilon)
        return left[:-1] + right
    return [start, end]


def _catmull_rom_point(p0, p1, p2, p3, t):
    t2, t3 = t * t, t * t * t
    x = 0.5 * (
        2 * p1[0]
        + (-p0[0] + p2[0]) * t
        + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
        + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3
    )
    y = 0.5 * (
        2 * p1[1]
        + (-p0[1] + p2[1]) * t
        + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
        + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3
    )
    return (x, y)


def build_smooth_path(points, samples_per_seg=15):
    """Catmull-Rom smoothing through a polyline (removes jitter/corners)."""
    if len(points) < 3:
        return points[:]
    padded = [points[0]] + points + [points[-1]]
    result = []
    for i in range(1, len(padded) - 2):
        p0, p1, p2, p3 = padded[i - 1], padded[i], padded[i + 1], padded[i + 2]
        for s in range(samples_per_seg):
            t = s / samples_per_seg
            result.append(_catmull_rom_point(p0, p1, p2, p3, t))
    result.append(points[-1])
    return result


def resample_by_distance(points, spacing):
    """Resample a polyline at (roughly) even arc-length spacing."""
    if len(points) < 2 or spacing <= 0:
        return points[:]
    cum = [0.0]
    for i in range(1, len(points)):
        cum.append(cum[-1] + dist(points[i], points[i - 1]))
    total = cum[-1]
    if total <= 0:
        return [points[0]]
    out = []
    j = 0
    n_steps = int(total / spacing) + 1
    for k in range(n_steps + 1):
        target = min(k * spacing, total)
        while j < len(cum) - 2 and cum[j + 1] < target:
            j += 1
        seg_len = cum[j + 1] - cum[j]
        t = 0.0 if seg_len == 0 else (target - cum[j]) / seg_len
        x = points[j][0] + (points[j + 1][0] - points[j][0]) * t
        y = points[j][1] + (points[j + 1][1] - points[j][1]) * t
        out.append((x, y))
        if target >= total:
            break
    return out


def trapezoidal_speeds(n, max_speed, ramp_fraction=0.18, min_speed=30):
    """Simple ease-in/ease-out speed profile for a pure-pursuit path file."""
    if n <= 1:
        return [max_speed] * n
    ramp_n = max(1, int(n * ramp_fraction))
    speeds = []
    for i in range(n):
        if i < ramp_n:
            f = (i + 1) / ramp_n
        elif i >= n - ramp_n:
            f = (n - i) / ramp_n
        else:
            f = 1.0
        f = max(0.0, min(1.0, f))
        speeds.append(round(min_speed + (max_speed - min_speed) * f))
    return speeds


def convert_units(points_m, unit):
    factor = UNIT_FACTORS.get(unit, 1.0)
    return [(x * factor, y * factor) for x, y in points_m]


# ---------------------------------------------------------------------------
# Code generators
# ---------------------------------------------------------------------------
def gen_lemlib_cpp(waypoints_m, settings):
    obj = settings["obj_name"] or "chassis"
    pts = convert_units(waypoints_m, settings["units"])
    lines = [
        "// Generated by VEX Path Planner - LemLib (PROS C++)",
        f"// Units: {settings['units']}  |  Track width: {settings['track_width']} in"
        f"  |  Wheel diameter: {settings['wheel_dia']} in",
        "// NOTE: field names in the param braces (maxSpeed, forwards, ...) follow the",
        "// LemLib MoveToPointParams / MoveToPoseParams structs - adjust to match your",
        "// installed LemLib version if it differs.",
        "",
        "void autonomous() {",
        f"    {obj}.setPose({pts[0][0]:.2f}, {pts[0][1]:.2f}, {settings['start_heading']:.1f});"
        if pts else f"    {obj}.setPose(0, 0, {settings['start_heading']:.1f});",
    ]
    for i, (x, y) in enumerate(pts):
        is_last = i == len(pts) - 1
        if is_last and len(pts) > 1:
            dx = pts[i][0] - pts[i - 1][0]
            dy = pts[i][1] - pts[i - 1][1]
            hdg = heading_deg(dx, dy)
            lines.append(
                f"    {obj}.moveToPose({x:.2f}, {y:.2f}, {hdg:.1f}, {settings['timeout']}, "
                f"{{.maxSpeed = {settings['max_speed']}}});"
            )
        else:
            lines.append(
                f"    {obj}.moveToPoint({x:.2f}, {y:.2f}, {settings['timeout']}, "
                f"{{.maxSpeed = {settings['max_speed']}}});"
            )
    lines.append(f"    {obj}.waitUntilDone();")
    lines.append("}")
    return "\n".join(lines)


def gen_lemlib_path_file(smooth_points_m, settings):
    pts = convert_units(smooth_points_m, settings["units"])
    speeds = trapezoidal_speeds(len(pts), settings["max_speed"])
    lines = [
        "# Generated by VEX Path Planner - LemLib pure-pursuit path asset",
        f"# Units: {settings['units']}  |  Save this as e.g. static/paths/myPath.txt",
        "# and load it in code with: ASSET(myPath_txt)  ->  chassis.follow(myPath_txt, lookahead, timeout);",
    ]
    for (x, y), s in zip(pts, speeds):
        lines.append(f"{x:.3f}, {y:.3f}, {s}")
    lines.append("endData")
    return "\n".join(lines)


def gen_vexcode_cpp(waypoints_m, settings):
    obj = settings["obj_name"] or "Drivetrain"
    pts = convert_units(waypoints_m, settings["units"])
    lines = [
        "// Generated by VEX Path Planner - VEXcode V5 C++ (turn + drive)",
        "// Requires: using namespace vex;   (generated by the VEXcode project template)",
        f"// Units: {settings['units']}",
        "",
        "void autonomous(void) {",
        f"    {obj}.setHeading({settings['start_heading']:.1f}, degrees);",
    ]
    heading = settings["start_heading"]
    for i in range(1, len(pts)):
        x0, y0 = pts[i - 1]
        x1, y1 = pts[i]
        dx, dy = x1 - x0, y1 - y0
        d = math.hypot(dx, dy)
        hdg = heading_deg(dx, dy)
        unit_name = "inches" if settings["units"] == "in" else settings["units"]
        lines.append(f"    {obj}.turnToHeading({hdg:.1f}, degrees);")
        lines.append(f"    {obj}.driveFor(forward, {d:.2f}, {unit_name});")
        heading = hdg
    lines.append("}")
    return "\n".join(lines)


def gen_vexcode_python(waypoints_m, settings):
    obj = settings["obj_name"] or "drivetrain"
    pts = convert_units(waypoints_m, settings["units"])
    unit_const = {"in": "INCHES", "mm": "MM", "cm": "CM", "m": "MM"}[settings["units"]]
    lines = [
        "# Generated by VEX Path Planner - VEXcode V5 Python (turn + drive)",
        "# Requires: from vex import *   (included by the VEXcode project template)",
        f"# Units: {settings['units']}",
        "",
        "def autonomous():",
        f"    {obj}.set_heading({settings['start_heading']:.1f}, DEGREES)",
    ]
    for i in range(1, len(pts)):
        x0, y0 = pts[i - 1]
        x1, y1 = pts[i]
        dx, dy = x1 - x0, y1 - y0
        d = math.hypot(dx, dy)
        hdg = heading_deg(dx, dy)
        lines.append(f"    {obj}.turn_to_heading({hdg:.1f}, DEGREES)")
        lines.append(f"    {obj}.drive_for(FORWARD, {d:.2f}, {unit_const})")
    return "\n".join(lines)


TARGETS = {
    "LemLib (PROS C++) - moveToPoint/Pose": ("cpp", gen_lemlib_cpp, "waypoints"),
    "LemLib Path File (.txt, pure pursuit)": ("txt", gen_lemlib_path_file, "smooth"),
    "VEXcode V5 C++ (turn + drive)": ("cpp", gen_vexcode_cpp, "waypoints"),
    "VEXcode V5 Python (turn + drive)": ("py", gen_vexcode_python, "waypoints"),
}


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------
class PathPlannerApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("VEX Autonomous Path Planner")
        self.geometry(f"{CANVAS_PX + 380}x{CANVAS_PX + 60}")
        self.minsize(1000, 700)

        self.waypoints_m = []      
        self.dragging_index = None

        self.simplified_m = []     
        self.smooth_m = []         

        self._build_layout()
        self._draw_static_field()
        self.update_path()

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------
    def _build_layout(self):
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # ---- left: canvas ----
        left = ctk.CTkFrame(self)
        left.grid(row=0, column=0, padx=10, pady=10, sticky="ns")

        info = ctk.CTkLabel(
            left,
            text="Left-click: add / drag point   |   Click near a line: insert point   |   Right-click: delete point",
            font=("Segoe UI", 12),
        )
        info.pack(pady=(4, 6))

        self.canvas = tk.Canvas(left, width=CANVAS_PX, height=CANVAS_PX,
                                 bg="#1e1e1e", highlightthickness=0)
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self.on_left_click)
        self.canvas.bind("<B1-Motion>", self.on_left_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_left_release)
        self.canvas.bind("<Button-3>", self.on_right_click)

        self.status_label = ctk.CTkLabel(left, text="", font=("Segoe UI", 12))
        self.status_label.pack(pady=(6, 0))

        # ---- right: controls ----
        right = ctk.CTkScrollableFrame(self, width=350, label_text="Path Controls")
        right.grid(row=0, column=1, padx=(0, 10), pady=10, sticky="nsew")

        # Path tools
        self._section(right, "Path")
        row = ctk.CTkFrame(right, fg_color="transparent")
        row.pack(fill="x", pady=4)
        ctk.CTkButton(row, text="Undo Point", command=self.undo_point).pack(side="left", expand=True, fill="x", padx=2)
        ctk.CTkButton(row, text="Clear Path", command=self.clear_path).pack(side="left", expand=True, fill="x", padx=2)

        row2 = ctk.CTkFrame(right, fg_color="transparent")
        row2.pack(fill="x", pady=4)
        ctk.CTkButton(row2, text="Save Path (.json)", command=self.save_path).pack(side="left", expand=True, fill="x", padx=2)
        ctk.CTkButton(row2, text="Load Path (.json)", command=self.load_path).pack(side="left", expand=True, fill="x", padx=2)

        # Simplification
        self._section(right, "Simplify / Smooth")
        self.epsilon_var = tk.DoubleVar(value=0.05)
        self._slider(right, "RDP tolerance (m) - higher = fewer corners", self.epsilon_var, 0.0, 0.3, self.update_path)

        self.smooth_res_var = tk.IntVar(value=15)
        self._slider(right, "Curve smoothness (samples/segment)", self.smooth_res_var, 3, 40, self.update_path, is_int=True)

        self.pp_spacing_var = tk.DoubleVar(value=0.05)
        self._slider(right, "Pure-pursuit point spacing (m)", self.pp_spacing_var, 0.02, 0.3, self.update_path)

        # Robot settings
        self._section(right, "Robot Settings")
        self.track_width_e = self._entry(right, "Track width (in)", "12.5")
        self.wheel_dia_e = self._entry(right, "Wheel diameter (in)", "3.25")
        self.start_heading_e = self._entry(right, "Start heading (deg, 0=+Y/north, CW+)", "0")
        self.max_speed_e = self._entry(right, "Max speed (0-127)", "100")
        self.timeout_e = self._entry(right, "Move timeout (ms)", "3000")
        self.obj_name_e = self._entry(right, "Drivetrain object name", "chassis")

        self.units_var = tk.StringVar(value="in")
        lbl = ctk.CTkLabel(right, text="Coordinate units in generated code")
        lbl.pack(anchor="w", padx=6, pady=(8, 0))
        ctk.CTkOptionMenu(right, values=list(UNIT_FACTORS.keys()), variable=self.units_var).pack(fill="x", padx=6, pady=(2, 6))

        # Code generation
        self._section(right, "Generate Code")
        self.target_var = tk.StringVar(value=list(TARGETS.keys())[0])
        ctk.CTkOptionMenu(right, values=list(TARGETS.keys()), variable=self.target_var,
                           command=self._on_target_change).pack(fill="x", padx=6, pady=(2, 6))
        ctk.CTkButton(right, text="Generate Code", command=self.generate_code,
                      fg_color="#2e7d32", hover_color="#1b5e20").pack(fill="x", padx=6, pady=(4, 10))

    def _section(self, parent, text):
        ctk.CTkLabel(parent, text=text, font=("Segoe UI", 15, "bold")).pack(anchor="w", padx=6, pady=(14, 4))

    def _entry(self, parent, label, default):
        ctk.CTkLabel(parent, text=label).pack(anchor="w", padx=6, pady=(4, 0))
        e = ctk.CTkEntry(parent)
        e.insert(0, default)
        e.pack(fill="x", padx=6, pady=(0, 2))
        return e

    def _slider(self, parent, label, var, frm, to, callback, is_int=False):
        wrap = ctk.CTkFrame(parent, fg_color="transparent")
        wrap.pack(fill="x", padx=6, pady=(4, 2))
        lbl = ctk.CTkLabel(wrap, text=f"{label}: {var.get()}")
        lbl.pack(anchor="w")

        def on_change(v):
            val = int(float(v)) if is_int else round(float(v), 3)
            var.set(val)
            lbl.configure(text=f"{label}: {val}")
            callback()

        slider = ctk.CTkSlider(wrap, from_=frm, to=to, variable=var, command=on_change)
        slider.pack(fill="x")

    def _on_target_change(self, choice):
        # Suggest a sensible default object name per target
        defaults = {
            "LemLib (PROS C++) - moveToPoint/Pose": "chassis",
            "LemLib Path File (.txt, pure pursuit)": "chassis",
            "VEXcode V5 C++ (turn + drive)": "Drivetrain",
            "VEXcode V5 Python (turn + drive)": "drivetrain",
        }
        current = self.obj_name_e.get().strip()
        if current in ("", "chassis", "Drivetrain", "drivetrain"):
            self.obj_name_e.delete(0, "end")
            self.obj_name_e.insert(0, defaults.get(choice, "chassis"))

    # ------------------------------------------------------------------
    # Field drawing
    # ------------------------------------------------------------------
    def _draw_static_field(self):
        c = self.canvas
        n_tiles = int(round(FIELD_SIZE_M / TILE_SIZE_M))
        colors = ["#8e8e8e", "#7f7f7f"]
        for row in range(n_tiles):
            for col in range(n_tiles):
                x0 = MARGIN_PX + col * (FIELD_PX / n_tiles)
                y0 = MARGIN_PX + row * (FIELD_PX / n_tiles)
                x1 = x0 + FIELD_PX / n_tiles
                y1 = y0 + FIELD_PX / n_tiles
                color = colors[(row + col) % 2]
                c.create_rectangle(x0, y0, x1, y1, fill=color, outline="#5a5a5a", width=1)

        c.create_rectangle(MARGIN_PX, MARGIN_PX, MARGIN_PX + FIELD_PX, MARGIN_PX + FIELD_PX,
                            outline="#cfcfcf", width=3)

        # axes
        x0, _ = m_to_px(-FIELD_SIZE_M / 2 - 0.15, 0)
        x1, _ = m_to_px(FIELD_SIZE_M / 2 + 0.15, 0)
        _, y0 = m_to_px(0, FIELD_SIZE_M / 2 + 0.15)
        _, y1 = m_to_px(0, -FIELD_SIZE_M / 2 - 0.15)
        cy = m_to_px(0, 0)[1]
        cx = m_to_px(0, 0)[0]
        c.create_line(x0, cy, x1, cy, fill="#e53935", width=3, arrow=tk.BOTH)
        c.create_line(cx, y0, cx, y1, fill="#3f6bff", width=3, arrow=tk.BOTH)
        c.create_text(x1 + 14, cy, text="+X", fill="#e53935", font=("Segoe UI", 11, "bold"))
        c.create_text(x0 - 14, cy, text="-X", fill="#e53935", font=("Segoe UI", 11, "bold"))
        c.create_text(cx, y0 - 12, text="+Y", fill="#3f6bff", font=("Segoe UI", 11, "bold"))
        c.create_text(cx, y1 + 12, text="-Y", fill="#3f6bff", font=("Segoe UI", 11, "bold"))

        # ticks / labels every TILE_SIZE_M
        steps = int(round((FIELD_SIZE_M / 2) / TILE_SIZE_M))
        for i in range(-steps, steps + 1):
            if i == 0:
                continue
            v = round(i * TILE_SIZE_M, 2)
            px, py = m_to_px(v, 0)
            c.create_line(px, cy - 6, px, cy + 6, fill="#e53935", width=2)
            c.create_text(px, cy + 18, text=f"{v} m", fill="white", font=("Segoe UI", 9))
            px2, py2 = m_to_px(0, v)
            c.create_line(cx - 6, py2, cx + 6, py2, fill="#3f6bff", width=2)
            c.create_text(cx + 34, py2, text=f"{v} m", fill="white", font=("Segoe UI", 9))

    # ------------------------------------------------------------------
    # Mouse interaction
    # ------------------------------------------------------------------
    def _nearest_point_index(self, px, py):
        best_i, best_d = None, POINT_HIT_PX
        for i, (x, y) in enumerate(self.waypoints_m):
            qx, qy = m_to_px(x, y)
            d = math.hypot(qx - px, qy - py)
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def _nearest_segment_index(self, px, py):
        """Return index i such that click is near segment (i, i+1), else None."""
        if len(self.waypoints_m) < 2:
            return None
        best_i, best_d = None, SEGMENT_HIT_PX
        for i in range(len(self.waypoints_m) - 1):
            a = m_to_px(*self.waypoints_m[i])
            b = m_to_px(*self.waypoints_m[i + 1])
            d = point_segment_distance((px, py), a, b)
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def on_left_click(self, event):
        idx = self._nearest_point_index(event.x, event.y)
        if idx is not None:
            self.dragging_index = idx
            return

        seg_idx = self._nearest_segment_index(event.x, event.y)
        x_m, y_m = px_to_m(event.x, event.y)
        # clamp inside field
        half = FIELD_SIZE_M / 2
        x_m = max(-half, min(half, x_m))
        y_m = max(-half, min(half, y_m))

        if seg_idx is not None:
            self.waypoints_m.insert(seg_idx + 1, (x_m, y_m))
        else:
            self.waypoints_m.append((x_m, y_m))
        self.update_path()

    def on_left_drag(self, event):
        if self.dragging_index is None:
            return
        x_m, y_m = px_to_m(event.x, event.y)
        half = FIELD_SIZE_M / 2
        x_m = max(-half, min(half, x_m))
        y_m = max(-half, min(half, y_m))
        self.waypoints_m[self.dragging_index] = (x_m, y_m)
        self.update_path()

    def on_left_release(self, event):
        self.dragging_index = None

    def on_right_click(self, event):
        idx = self._nearest_point_index(event.x, event.y)
        if idx is not None:
            self.waypoints_m.pop(idx)
            self.update_path()

    # ------------------------------------------------------------------
    # Path buttons
    # ------------------------------------------------------------------
    def undo_point(self):
        if self.waypoints_m:
            self.waypoints_m.pop()
            self.update_path()

    def clear_path(self):
        self.waypoints_m = []
        self.update_path()

    def save_path(self):
        if not self.waypoints_m:
            messagebox.showinfo("Save Path", "No points to save yet.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                             filetypes=[("JSON", "*.json")])
        if not path:
            return
        with open(path, "w") as f:
            json.dump({"waypoints_m": self.waypoints_m}, f, indent=2)
        messagebox.showinfo("Save Path", f"Saved to {path}")

    def load_path(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return
        try:
            with open(path) as f:
                data = json.load(f)
            self.waypoints_m = [tuple(p) for p in data["waypoints_m"]]
            self.update_path()
        except Exception as e:
            messagebox.showerror("Load Path", f"Could not load file:\n{e}")

    # ------------------------------------------------------------------
    # Recompute + redraw
    # ------------------------------------------------------------------
    def update_path(self):
        eps = self.epsilon_var.get()
        self.simplified_m = rdp(self.waypoints_m, eps) if len(self.waypoints_m) >= 2 else self.waypoints_m[:]
        samples = int(self.smooth_res_var.get())
        self.smooth_m = build_smooth_path(self.simplified_m, samples_per_seg=samples)
        self._redraw_dynamic()
        self.status_label.configure(
            text=f"Raw points: {len(self.waypoints_m)}   |   Simplified: {len(self.simplified_m)}   |   Smoothed samples: {len(self.smooth_m)}"
        )

    def _redraw_dynamic(self):
        c = self.canvas
        c.delete("path")

        # raw path (thin dashed)
        if len(self.waypoints_m) >= 2:
            coords = []
            for x, y in self.waypoints_m:
                coords.extend(m_to_px(x, y))
            c.create_line(*coords, fill="#888888", width=1, dash=(3, 3), tags="path")

        # smoothed curve (final path)
        if len(self.smooth_m) >= 2:
            coords = []
            for x, y in self.smooth_m:
                coords.extend(m_to_px(x, y))
            c.create_line(*coords, fill="#ffb300", width=3, smooth=False, tags="path")

        # simplified corner markers
        for i, (x, y) in enumerate(self.simplified_m):
            px, py = m_to_px(x, y)
            c.create_oval(px - 5, py - 5, px + 5, py + 5, fill="#42a5f5", outline="white", tags="path")

        # raw waypoints (draggable handles)
        for i, (x, y) in enumerate(self.waypoints_m):
            px, py = m_to_px(x, y)
            color = "#00e676" if i == 0 else ("#ff5252" if i == len(self.waypoints_m) - 1 else "#ffffff")
            c.create_oval(px - 4, py - 4, px + 4, py + 4, fill=color, outline="black", tags="path")

    # ------------------------------------------------------------------
    # Settings collection + code generation
    # ------------------------------------------------------------------
    def _collect_settings(self):
        def f(entry, default):
            try:
                return float(entry.get())
            except ValueError:
                return default

        return {
            "track_width": f(self.track_width_e, 12.5),
            "wheel_dia": f(self.wheel_dia_e, 3.25),
            "start_heading": f(self.start_heading_e, 0.0),
            "max_speed": int(f(self.max_speed_e, 100)),
            "timeout": int(f(self.timeout_e, 3000)),
            "obj_name": self.obj_name_e.get().strip(),
            "units": self.units_var.get(),
        }

    def generate_code(self):
        if len(self.waypoints_m) < 2:
            messagebox.showinfo("Generate Code", "Draw at least 2 points on the field first.")
            return

        settings = self._collect_settings()
        target_name = self.target_var.get()
        ext, gen_fn, source = TARGETS[target_name]

        if source == "waypoints":
            data = self.simplified_m if self.simplified_m else self.waypoints_m
        else:  # "smooth" -> resample the smoothed curve at even spacing for pure pursuit
            spacing = self.pp_spacing_var.get()
            data = resample_by_distance(self.smooth_m, spacing)

        code = gen_fn(data, settings)
        self._show_code_window(target_name, code, ext)

    def _show_code_window(self, title, code, ext):
        win = ctk.CTkToplevel(self)
        win.title(f"Generated Code - {title}")
        win.geometry("760x600")

        box = ctk.CTkTextbox(win, font=("Consolas", 12))
        box.pack(fill="both", expand=True, padx=10, pady=(10, 4))
        box.insert("1.0", code)

        btn_row = ctk.CTkFrame(win, fg_color="transparent")
        btn_row.pack(fill="x", padx=10, pady=(0, 10))

        def copy():
            self.clipboard_clear()
            self.clipboard_append(code)

        def save():
            path = filedialog.asksaveasfilename(defaultextension=f".{ext}",
                                                 filetypes=[(ext.upper(), f"*.{ext}"), ("All files", "*.*")])
            if path:
                with open(path, "w") as fh:
                    fh.write(code)
                messagebox.showinfo("Save", f"Saved to {path}")

        ctk.CTkButton(btn_row, text="Copy to Clipboard", command=copy).pack(side="left", expand=True, fill="x", padx=4)
        ctk.CTkButton(btn_row, text="Save As...", command=save).pack(side="left", expand=True, fill="x", padx=4)


if __name__ == "__main__":
    app = PathPlannerApp()
    app.mainloop()
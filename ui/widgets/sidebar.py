import tkinter as tk

import customtkinter as ctk

from codegen.codegen import TARGET_NAMES
from config import DRIVETRAIN_LABEL_TO_KEY, DRIVETRAIN_TYPES, UNIT_FACTORS
from ui.widgets.inputs import LabeledEntry, LabeledSlider, SectionHeader

DEFAULT_OBJ_NAMES = {
    "LemLib (PROS C++) - moveToPoint/Pose": "chassis",
    "LemLib Path File (.txt, pure pursuit)": "chassis",
    "VEXcode V5 C++ (turn + drive)": "Drivetrain",
    "VEXcode V5 Python (turn + drive)": "drivetrain",
}

# Each entry: the frame attribute to show, and the note text for that drivetrain.
# tank_standard falls back to the tank frame + generic note if not listed.
DRIVETRAIN_NOTES = {
    "holonomic": "LemLib is tank-only and will ignore this (still generates tank code + a warning).",
    "tank_smart": "Needs an Inertial/GPS/Gyro sensor on the drivetrain.",
    "tank_standard": "Works with any drivetrain, no gyro needed.",
}


class Sidebar(ctk.CTkScrollableFrame):
    def __init__(self, master, on_recompute, on_generate, **kwargs):
        super().__init__(master, width=350, label_text="Path Controls", **kwargs)
        self.on_recompute = on_recompute
        self.on_generate = on_generate

        self._build_path_section()
        self._build_simplify_section()
        self._build_drivetrain_section()
        self._build_robot_settings_section()
        self._build_export_section()

        self._apply_drivetrain_selection(self.drivetrain_type_var.get())

    def _build_path_section(self):
        SectionHeader(self, "Path")

        row = ctk.CTkFrame(self, fg_color="transparent")
        row.pack(fill="x", pady=4)
        self.undo_btn = ctk.CTkButton(row, text="Undo Point")
        self.undo_btn.pack(side="left", expand=True, fill="x", padx=2)
        self.clear_btn = ctk.CTkButton(row, text="Clear Path")
        self.clear_btn.pack(side="left", expand=True, fill="x", padx=2)

        row2 = ctk.CTkFrame(self, fg_color="transparent")
        row2.pack(fill="x", pady=4)
        self.save_btn = ctk.CTkButton(row2, text="Save Path (.json)")
        self.save_btn.pack(side="left", expand=True, fill="x", padx=2)
        self.load_btn = ctk.CTkButton(row2, text="Load Path (.json)")
        self.load_btn.pack(side="left", expand=True, fill="x", padx=2)

    def _build_simplify_section(self):
        SectionHeader(self, "Simplify / Smooth")

        self.epsilon_var = tk.DoubleVar(value=0.05)
        LabeledSlider(self, "RDP tolerance (m) - higher = fewer corners", self.epsilon_var, 0.0, 0.3, self.on_recompute)

        self.smooth_res_var = tk.IntVar(value=15)
        LabeledSlider(self, "Curve smoothness (samples/segment)", self.smooth_res_var, 3, 40, self.on_recompute, is_int=True)

        self.pp_spacing_var = tk.DoubleVar(value=0.05)
        LabeledSlider(self, "Pure-pursuit point spacing (m)", self.pp_spacing_var, 0.02, 0.3, self.on_recompute)

    def _build_drivetrain_section(self):
        SectionHeader(self, "Drivetrain Type")

        self.drivetrain_type_var = tk.StringVar(value=DRIVETRAIN_TYPES["tank_standard"])
        ctk.CTkOptionMenu(
            self,
            values=list(DRIVETRAIN_TYPES.values()),
            variable=self.drivetrain_type_var,
            command=self._apply_drivetrain_selection,
        ).pack(fill="x", padx=6, pady=(2, 4))

        self.dt_note_label = ctk.CTkLabel(
            self, text="", font=("Segoe UI", 11), text_color="#ffb300",
            wraplength=320, justify="left", anchor="w"
        )
        self.dt_note_label.pack(fill="x", padx=6, pady=(0, 6))

    def _build_robot_settings_section(self):
        SectionHeader(self, "Robot Settings")

        self.track_width_e = LabeledEntry(self, "Track width (in)", "12.5")
        self.wheel_dia_e = LabeledEntry(self, "Wheel diameter (in)", "3.25")
        self.start_heading_e = LabeledEntry(self, "Start heading (deg, 0=+Y/north, CW+)", "0")
        self.max_speed_e = LabeledEntry(self, "Max speed (0-127)", "100")
        self.timeout_e = LabeledEntry(self, "Move timeout (ms)", "3000")

        self.dt_fields_container = ctk.CTkFrame(self, fg_color="transparent")
        self.dt_fields_container.pack(fill="x")

        self.tank_fields_frame = ctk.CTkFrame(self.dt_fields_container, fg_color="transparent")
        self.obj_name_e = LabeledEntry(self.tank_fields_frame, "Drivetrain object name", "chassis")

        self.holo_fields_frame = ctk.CTkFrame(self.dt_fields_container, fg_color="transparent")
        self.motor_fl_e = LabeledEntry(self.holo_fields_frame, "Front-Left motor name", "motor_fl")
        self.motor_fr_e = LabeledEntry(self.holo_fields_frame, "Front-Right motor name", "motor_fr")
        self.motor_bl_e = LabeledEntry(self.holo_fields_frame, "Back-Left motor name", "motor_bl")
        self.motor_br_e = LabeledEntry(self.holo_fields_frame, "Back-Right motor name", "motor_br")
        self.gear_ratio_e = LabeledEntry(self.holo_fields_frame, "External gear ratio", "1.0")

        # Frame swap is driven by _apply_drivetrain_selection; this just sets the
        # layout parent so pack() has somewhere to land before that first runs.
        self.tank_fields_frame.pack(fill="x")

        self.units_var = tk.StringVar(value="in")
        ctk.CTkLabel(self, text="Coordinate units in generated code").pack(anchor="w", padx=6, pady=(8, 0))
        ctk.CTkOptionMenu(self, values=list(UNIT_FACTORS.keys()), variable=self.units_var).pack(fill="x", padx=6, pady=(2, 6))

    def _build_export_section(self):
        SectionHeader(self, "Generate Code")

        self.target_var = tk.StringVar(value=TARGET_NAMES[0])
        ctk.CTkOptionMenu(
            self, values=TARGET_NAMES, variable=self.target_var, command=self._sync_obj_name_default
        ).pack(fill="x", padx=6, pady=(2, 6))

        ctk.CTkButton(
            self, text="Generate Code", command=lambda: self.on_generate(),
            fg_color="#2e7d32", hover_color="#1b5e20"
        ).pack(fill="x", padx=6, pady=(4, 10))

    def _sync_obj_name_default(self, choice):
        current = self.obj_name_e.get().strip()
        if current in ("", "chassis", "Drivetrain", "drivetrain"):
            self.obj_name_e.set(DEFAULT_OBJ_NAMES.get(choice, "chassis"))

    def _apply_drivetrain_selection(self, choice):
        key = DRIVETRAIN_LABEL_TO_KEY.get(choice, "tank_standard")
        is_holonomic = key == "holonomic"

        active_frame = self.holo_fields_frame if is_holonomic else self.tank_fields_frame
        inactive_frame = self.tank_fields_frame if is_holonomic else self.holo_fields_frame
        inactive_frame.pack_forget()
        active_frame.pack(fill="x")

        self.dt_note_label.configure(text=DRIVETRAIN_NOTES.get(key, DRIVETRAIN_NOTES["tank_standard"]))

        if not is_holonomic:
            self._sync_obj_name_default(self.target_var.get())

    def collect_settings(self):
        return {
            "track_width": self.track_width_e.get_float(12.5),
            "wheel_dia": self.wheel_dia_e.get_float(3.25),
            "start_heading": self.start_heading_e.get_float(0.0),
            "max_speed": int(self.max_speed_e.get_float(100)),
            "timeout": int(self.timeout_e.get_float(3000)),
            "obj_name": self.obj_name_e.get().strip(),
            "units": self.units_var.get(),
            "drivetrain_type": DRIVETRAIN_LABEL_TO_KEY.get(self.drivetrain_type_var.get(), "tank_standard"),
            "motor_fl": self.motor_fl_e.get().strip(),
            "motor_fr": self.motor_fr_e.get().strip(),
            "motor_bl": self.motor_bl_e.get().strip(),
            "motor_br": self.motor_br_e.get().strip(),
            "gear_ratio": self.gear_ratio_e.get_float(1.0),
        }

from tkinter import filedialog, messagebox

import customtkinter as ctk

from codegen.codegen import generate_code as run_codegen
from codegen.codegen import target_ext
from config import CANVAS_PX
from core.state_manager import PathState, load_path_file, save_path_file
from ui.widgets.code_window import CodeWindow
from ui.widgets.path_canvas import PathCanvas
from ui.widgets.sidebar import Sidebar

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


class PathPlannerApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("VEX Autonomous Path Planner")
        self.geometry(f"{CANVAS_PX + 380}x{CANVAS_PX + 60}")
        self.minsize(1000, 760)

        self.state_data = PathState()

        self._build_layout()
        self._wire_sidebar_actions()
        self._recompute_and_redraw()

    def _build_layout(self):
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        left = ctk.CTkFrame(self)
        left.grid(row=0, column=0, padx=10, pady=10, sticky="ns")

        info_text = "Left-click: add / drag point   |   Click near a line: insert point   |   Right-click: delete point"
        ctk.CTkLabel(left, text=info_text, font=("Segoe UI", 12)).pack(pady=(4, 6))

        self.canvas = PathCanvas(left, self.state_data, on_change=self._recompute_and_redraw)
        self.canvas.pack()

        self.status_label = ctk.CTkLabel(left, text="", font=("Segoe UI", 12))
        self.status_label.pack(pady=(6, 0))

        self.sidebar = Sidebar(self, on_recompute=self._recompute_and_redraw, on_generate=self.generate_code)
        self.sidebar.grid(row=0, column=1, padx=(0, 10), pady=10, sticky="nsew")

    def _wire_sidebar_actions(self):
        self.sidebar.undo_btn.configure(command=self._undo_point)
        self.sidebar.clear_btn.configure(command=self._clear_path)
        self.sidebar.save_btn.configure(command=self._save_path)
        self.sidebar.load_btn.configure(command=self._load_path)

    def _undo_point(self):
        self.state_data.undo()
        self._recompute_and_redraw()

    def _clear_path(self):
        self.state_data.clear()
        self._recompute_and_redraw()

    def _save_path(self):
        if not self.state_data.waypoints_m:
            messagebox.showinfo("Save Path", "No points to save yet.")
            return

        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
        if not path:
            return

        save_path_file(self.state_data, path)
        messagebox.showinfo("Save Path", f"Saved to {path}")

    def _load_path(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return

        try:
            load_path_file(self.state_data, path)
            self._recompute_and_redraw()
        except Exception as e:
            messagebox.showerror("Load Path", f"Could not load file:\n{e}")

    def _recompute_and_redraw(self):
        self.state_data.recompute(
            epsilon=self.sidebar.epsilon_var.get(),
            smooth_samples=int(self.sidebar.smooth_res_var.get()),
        )
        self.canvas.redraw_path()

        self.status_label.configure(
            text=f"Raw points: {len(self.state_data.waypoints_m)}   |   "
                 f"Simplified: {len(self.state_data.simplified_m)}   |   "
                 f"Smoothed samples: {len(self.state_data.smooth_m)}"
        )

    def generate_code(self):
        if len(self.state_data.waypoints_m) < 2:
            messagebox.showinfo("Generate Code", "Draw at least 2 points on the field first.")
            return

        settings = self.sidebar.collect_settings()
        target_name = self.sidebar.target_var.get()
        waypoints = self.state_data.simplified_m or self.state_data.waypoints_m

        code = run_codegen(
            target_name, waypoints, self.state_data.smooth_m, settings, self.sidebar.pp_spacing_var.get()
        )
        CodeWindow(self, target_name, code, target_ext(target_name))

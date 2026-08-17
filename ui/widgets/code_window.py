from tkinter import filedialog, messagebox

import customtkinter as ctk


class CodeWindow(ctk.CTkToplevel):
    def __init__(self, master, title, code, ext):
        super().__init__(master)
        self.code = code
        self.ext = ext

        self.title(f"Generated Code - {title}")
        self.geometry("760x600")

        box = ctk.CTkTextbox(self, font=("Consolas", 12))
        box.pack(fill="both", expand=True, padx=10, pady=(10, 4))
        box.insert("1.0", code)

        btn_row = ctk.CTkFrame(self, fg_color="transparent")
        btn_row.pack(fill="x", padx=10, pady=(0, 10))

        ctk.CTkButton(btn_row, text="Copy to Clipboard", command=self._copy).pack(side="left", expand=True, fill="x", padx=4)
        ctk.CTkButton(btn_row, text="Save As...", command=self._save).pack(side="left", expand=True, fill="x", padx=4)

    def _copy(self):
        self.clipboard_clear()
        self.clipboard_append(self.code)

    def _save(self):
        path = filedialog.asksaveasfilename(
            defaultextension=f".{self.ext}",
            filetypes=[(self.ext.upper(), f"*.{self.ext}"), ("All files", "*.*")]
        )
        if path:
            with open(path, "w") as fh:
                fh.write(self.code)
            messagebox.showinfo("Save", f"Saved to {path}")

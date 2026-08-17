import customtkinter as ctk


class SectionHeader(ctk.CTkLabel):
    def __init__(self, master, text, **kwargs):
        super().__init__(master, text=text, font=("Segoe UI", 15, "bold"), **kwargs)
        self.pack(anchor="w", padx=6, pady=(14, 4))


class LabeledEntry(ctk.CTkFrame):
    def __init__(self, master, label, default="", **kwargs):
        super().__init__(master, fg_color="transparent", **kwargs)
        ctk.CTkLabel(self, text=label).pack(anchor="w", padx=6, pady=(4, 0))
        self.entry = ctk.CTkEntry(self)
        self.entry.insert(0, default)
        self.entry.pack(fill="x", padx=6, pady=(0, 2))
        self.pack(fill="x")

    def get(self):
        return self.entry.get()

    def set(self, value):
        self.entry.delete(0, "end")
        self.entry.insert(0, value)

    def get_float(self, default):
        try:
            return float(self.entry.get())
        except ValueError:
            return default


class LabeledSlider(ctk.CTkFrame):
    def __init__(self, master, label, variable, from_, to, on_change, is_int=False, **kwargs):
        super().__init__(master, fg_color="transparent", **kwargs)
        self._label = label
        self._var = variable
        self._is_int = is_int
        self._on_change = on_change

        self.value_label = ctk.CTkLabel(self, text=f"{label}: {variable.get()}")
        self.value_label.pack(anchor="w")

        self.slider = ctk.CTkSlider(self, from_=from_, to=to, variable=variable, command=self._handle_move)
        self.slider.pack(fill="x")
        self.pack(fill="x", padx=6, pady=(4, 2))

    def _handle_move(self, raw_value):
        value = int(float(raw_value)) if self._is_int else round(float(raw_value), 3)
        self._var.set(value)
        self.value_label.configure(text=f"{self._label}: {value}")
        self._on_change()

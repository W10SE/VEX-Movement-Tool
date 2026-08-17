import json

from core.algorithms import build_smooth_path, rdp


class PathState:
    def __init__(self):
        self.waypoints_m = []
        self.simplified_m = []
        self.smooth_m = []

    def add_point(self, point, index=None):
        if index is None:
            self.waypoints_m.append(point)
        else:
            self.waypoints_m.insert(index, point)

    def move_point(self, index, point):
        self.waypoints_m[index] = point

    def remove_point(self, index):
        self.waypoints_m.pop(index)

    def undo(self):
        if self.waypoints_m:
            self.waypoints_m.pop()

    def clear(self):
        self.waypoints_m = []
        self.simplified_m = []
        self.smooth_m = []

    def recompute(self, epsilon, smooth_samples):
        self.simplified_m = rdp(self.waypoints_m, epsilon) if len(self.waypoints_m) >= 2 else self.waypoints_m[:]
        self.smooth_m = build_smooth_path(self.simplified_m, samples_per_seg=smooth_samples)

    def to_dict(self):
        return {"waypoints_m": self.waypoints_m}

    def load_dict(self, data):
        self.waypoints_m = [tuple(p) for p in data.get("waypoints_m", [])]


def save_path_file(state, path):
    with open(path, "w") as f:
        json.dump(state.to_dict(), f, indent=2)


def load_path_file(state, path):
    with open(path) as f:
        data = json.load(f)
    state.load_dict(data)

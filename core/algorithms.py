#I would like to acknowledge that part of the code is not made by me, but AI.
from core.geometry import catmull_rom_point, dist, point_segment_distance


def rdp(points, epsilon):
    if len(points) < 3:
        return points[:]

    first, last = points[0], points[-1]
    farthest_gap, split_at = 0.0, 0

    for candidate in range(1, len(points) - 1):
        gap = point_segment_distance(points[candidate], first, last)
        if gap > farthest_gap:
            farthest_gap, split_at = gap, candidate

    if farthest_gap > epsilon:
        head = rdp(points[: split_at + 1], epsilon)
        tail = rdp(points[split_at:], epsilon)
        return head[:-1] + tail

    return [first, last]


def build_smooth_path(points, samples_per_seg=15):
    if len(points) < 3:
        return points[:]

    padded = [points[0]] + points + [points[-1]]
    curve = []

    for i in range(1, len(padded) - 2):
        before, start, end, after = padded[i - 1], padded[i], padded[i + 1], padded[i + 2]
        for step in range(samples_per_seg):
            frac = step / samples_per_seg
            curve.append(catmull_rom_point(before, start, end, after, frac))

    curve.append(points[-1])
    return curve


def resample_by_distance(points, spacing):
    if len(points) < 2 or spacing <= 0:
        return points[:]

    running_len = [0.0]
    for i in range(1, len(points)):
        running_len.append(running_len[-1] + dist(points[i], points[i - 1]))

    total_len = running_len[-1]
    if total_len <= 0:
        return [points[0]]

    resampled = []
    seg = 0
    step_count = int(total_len / spacing) + 1

    for k in range(step_count + 1):
        target = min(k * spacing, total_len)
        while seg < len(running_len) - 2 and running_len[seg + 1] < target:
            seg += 1

        seg_len = running_len[seg + 1] - running_len[seg]
        frac = 0.0 if seg_len == 0 else (target - running_len[seg]) / seg_len

        x = points[seg][0] + (points[seg + 1][0] - points[seg][0]) * frac
        y = points[seg][1] + (points[seg + 1][1] - points[seg][1]) * frac
        resampled.append((x, y))

        if target >= total_len:
            break

    return resampled


def trapezoidal_speeds(n, max_speed, ramp_fraction=0.18, min_speed=30):
    if n <= 1:
        return [max_speed] * n

    ramp_len = max(1, int(n * ramp_fraction))
    speeds = []

    for i in range(n):
        if i < ramp_len:
            scale = (i + 1) / ramp_len
        elif i >= n - ramp_len:
            scale = (n - i) / ramp_len
        else:
            scale = 1.0

        scale = max(0.0, min(1.0, scale))
        speeds.append(round(min_speed + (max_speed - min_speed) * scale))

    return speeds

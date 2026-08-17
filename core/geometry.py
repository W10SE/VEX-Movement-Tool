import math


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_segment_distance(p, seg_start, seg_end):
    if seg_start == seg_end:
        return dist(p, seg_start)

    ax, ay = seg_start
    bx, by = seg_end
    px, py = p

    seg_dx, seg_dy = bx - ax, by - ay
    projection = ((px - ax) * seg_dx + (py - ay) * seg_dy) / (seg_dx * seg_dx + seg_dy * seg_dy)
    projection = max(0.0, min(1.0, projection))

    closest_x, closest_y = ax + projection * seg_dx, ay + projection * seg_dy
    return math.hypot(px - closest_x, py - closest_y)


def catmull_rom_point(before, start, end, after, frac): #done using AI
    f2, f3 = frac * frac, frac * frac * frac
    x = 0.5 * (
        2 * start[0]
        + (-before[0] + end[0]) * frac
        + (2 * before[0] - 5 * start[0] + 4 * end[0] - after[0]) * f2
        + (-before[0] + 3 * start[0] - 3 * end[0] + after[0]) * f3
    )
    y = 0.5 * (
        2 * start[1]
        + (-before[1] + end[1]) * frac
        + (2 * before[1] - 5 * start[1] + 4 * end[1] - after[1]) * f2
        + (-before[1] + 3 * start[1] - 3 * end[1] + after[1]) * f3
    )
    return (x, y)

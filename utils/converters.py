import math

from config import CANVAS_PX, SCALE, UNIT_FACTORS


def m_to_px(x_m, y_m):
    px = CANVAS_PX / 2 + x_m * SCALE
    py = CANVAS_PX / 2 - y_m * SCALE
    return px, py


def px_to_m(px, py):
    x_m = (px - CANVAS_PX / 2) / SCALE
    y_m = -(py - CANVAS_PX / 2) / SCALE
    return x_m, y_m


def inches_to_unit(value_in, unit):
    return value_in / UNIT_FACTORS["in"] * UNIT_FACTORS.get(unit, 1.0)


def convert_units(points_m, unit):
    factor = UNIT_FACTORS.get(unit, 1.0)
    return [(x * factor, y * factor) for x, y in points_m]


def heading_deg(dx, dy):
    return math.degrees(math.atan2(dx, dy)) % 360.0


def signed_turn(current_heading, target_heading):
    diff = (target_heading - current_heading + 180.0) % 360.0 - 180.0
    if diff >= 0:
        return "RIGHT", diff
    return "LEFT", -diff

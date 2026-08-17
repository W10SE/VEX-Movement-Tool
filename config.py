FIELD_SIZE_M = 3.6
TILE_SIZE_M = 0.6
CANVAS_PX = 700
MARGIN_PX = 46
FIELD_PX = CANVAS_PX - 2 * MARGIN_PX
SCALE = FIELD_PX / FIELD_SIZE_M

UNIT_FACTORS = {"in": 39.37007874, "mm": 1000.0, "cm": 100.0, "m": 1.0}

POINT_HIT_PX = 10
SEGMENT_HIT_PX = 12

DRIVETRAIN_TYPES = {
    "tank_standard": "Tank / Differential - No Sensor",
    "tank_smart": "Tank / Differential - SmartDrive (Inertial/Gyro/GPS)",
    "holonomic": "Holonomic / Mecanum / X-Drive (4-motor)",
}
DRIVETRAIN_LABEL_TO_KEY = {label: key for key, label in DRIVETRAIN_TYPES.items()}

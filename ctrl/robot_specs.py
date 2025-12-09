import math

# Joint limits for KUKA KR120 R2700-2 (radians)
JOINT_LIMITS_RAD = (
    (math.radians(-185.0), math.radians(185.0)),   # A1
    (math.radians(-140.0), math.radians(-5.0)),    # A2
    (math.radians(-120.0), math.radians(168.0)),   # A3
    (math.radians(-350.0), math.radians(350.0)),   # A4
    (math.radians(-125.0), math.radians(125.0)),   # A5
    (math.radians(-350.0), math.radians(350.0)),   # A6
)

# Optional rated speeds (deg/s) from datasheet, kept for reference
JOINT_SPEEDS_DEG_S = (120.0, 115.0, 120.0, 190.0, 180.0, 260.0)


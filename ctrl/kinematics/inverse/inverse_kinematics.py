from ...postypes.configuration import configuration
from ...robot_specs import JOINT_LIMITS_RAD
import math


class InvKinematics:
    _ALPHA = (0.0, -math.pi / 2, 0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, 0.0)
    _A = (0.0, 0.35, 1.25, -0.054, 0.0, 0.0, 0.0)
    _D = (0.75, 0.0, 0.0, 1.5, 0.0, 0.0, 0.303)
    _THETA_OFFSET = (0.0, -math.pi / 2, 0.0, 0.0, 0.0, 0.0)
    _EPS = 1e-8

    @staticmethod
    def _dh_transform(alpha: float, a: float, d: float, theta: float):
        sa, ca = math.sin(alpha), math.cos(alpha)
        st, ct = math.sin(theta), math.cos(theta)
        return (
            (ct, -st, 0.0, a),
            (st * ca, ct * ca, -sa, -sa * d),
            (st * sa, ct * sa, ca, ca * d),
            (0.0, 0.0, 0.0, 1.0),
        )

    @staticmethod
    def _matmul(a, b):
        a00, a01, a02, a03 = a[0]
        a10, a11, a12, a13 = a[1]
        a20, a21, a22, a23 = a[2]
        a30, a31, a32, a33 = a[3]

        b00, b01, b02, b03 = b[0]
        b10, b11, b12, b13 = b[1]
        b20, b21, b22, b23 = b[2]
        b30, b31, b32, b33 = b[3]

        return (
            (
                a00 * b00 + a01 * b10 + a02 * b20 + a03 * b30,
                a00 * b01 + a01 * b11 + a02 * b21 + a03 * b31,
                a00 * b02 + a01 * b12 + a02 * b22 + a03 * b32,
                a00 * b03 + a01 * b13 + a02 * b23 + a03 * b33,
            ),
            (
                a10 * b00 + a11 * b10 + a12 * b20 + a13 * b30,
                a10 * b01 + a11 * b11 + a12 * b21 + a13 * b31,
                a10 * b02 + a11 * b12 + a12 * b22 + a13 * b32,
                a10 * b03 + a11 * b13 + a12 * b23 + a13 * b33,
            ),
            (
                a20 * b00 + a21 * b10 + a22 * b20 + a23 * b30,
                a20 * b01 + a21 * b11 + a22 * b21 + a23 * b31,
                a20 * b02 + a21 * b12 + a22 * b22 + a23 * b32,
                a20 * b03 + a21 * b13 + a22 * b23 + a23 * b33,
            ),
            (
                a30 * b00 + a31 * b10 + a32 * b20 + a33 * b30,
                a30 * b01 + a31 * b11 + a32 * b21 + a33 * b31,
                a30 * b02 + a31 * b12 + a32 * b22 + a33 * b32,
                a30 * b03 + a31 * b13 + a32 * b23 + a33 * b33,
            ),
        )

    @staticmethod
    def _rotation_matrix(roll: float, pitch: float, yaw: float):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )

    @staticmethod
    def _transpose3(m):
        return (
            (m[0][0], m[1][0], m[2][0]),
            (m[0][1], m[1][1], m[2][1]),
            (m[0][2], m[1][2], m[2][2]),
        )

    @staticmethod
    def _matmul3(a, b):
        return (
            (
                a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
                a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
                a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
            ),
            (
                a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
                a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
                a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
            ),
            (
                a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
                a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
                a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
            ),
        )

    @staticmethod
    def _clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))

    @staticmethod
    def _within_limits(joints):
        if len(joints) != len(JOINT_LIMITS_RAD):
            return False
        for value, (j_min, j_max) in zip(joints, JOINT_LIMITS_RAD):
            if value < j_min - InvKinematics._EPS or value > j_max + InvKinematics._EPS:
                return False
        return True

    def get_inv_kinematics(self, _pos):
        if _pos is None:
            return []

        if hasattr(_pos, "get_position"):
            px, py, pz, roll, pitch, yaw = _pos.get_position()
        else:
            px, py, pz, roll, pitch, yaw = _pos

        R0_6 = self._rotation_matrix(roll, pitch, yaw)
        d7 = self._D[6]
        nx, ny, nz = R0_6[0][2], R0_6[1][2], R0_6[2][2]
        wx = px - d7 * nx
        wy = py - d7 * ny
        wz = pz - d7 * nz

        theta1 = math.atan2(wy, wx)

        a1 = self._A[1]
        d1 = self._D[0]
        r = math.hypot(wx, wy) - a1
        s = wz - d1
        side_c = math.hypot(r, s)
        if side_c < self._EPS:
            return []

        side_a = math.hypot(self._A[3], self._D[3])
        side_b = self._A[2]
        angle_a = math.acos(self._clamp((side_b ** 2 + side_c ** 2 - side_a ** 2) / (2 * side_b * side_c), -1.0, 1.0))
        angle_b = math.acos(self._clamp((side_a ** 2 + side_c ** 2 - side_b ** 2) / (2 * side_a * side_c), -1.0, 1.0))
        theta2 = math.pi / 2 - angle_a - math.atan2(s, r)
        theta3 = math.pi / 2 - (angle_b + math.atan2(abs(self._A[3]), self._D[3]))

        T0_1 = self._dh_transform(self._ALPHA[0], self._A[0], self._D[0], theta1 + self._THETA_OFFSET[0])
        T1_2 = self._dh_transform(self._ALPHA[1], self._A[1], self._D[1], theta2 + self._THETA_OFFSET[1])
        T2_3 = self._dh_transform(self._ALPHA[2], self._A[2], self._D[2], theta3 + self._THETA_OFFSET[2])
        T0_2 = self._matmul(T0_1, T1_2)
        T0_3 = self._matmul(T0_2, T2_3)
        R0_3 = tuple(row[:3] for row in T0_3[:3])
        R3_6 = self._matmul3(self._transpose3(R0_3), R0_6)

        r03_02 = R3_6[0][2]
        r13_02 = R3_6[1][2]
        r23_02 = R3_6[2][2]

        sin_theta5_abs = math.sqrt(r03_02 * r03_02 + r23_02 * r23_02)
        if sin_theta5_abs < 1e-6:
            theta5 = 0.0 if r13_02 >= 0.0 else math.pi
            theta4 = 0.0
            theta6 = math.atan2(-R3_6[0][1], R3_6[0][0])
        else:
            theta4 = math.atan2(r23_02, -r03_02)
            if abs(math.cos(theta4)) > self._EPS:
                sin_theta5 = -r03_02 / math.cos(theta4)
            else:
                sin_theta5 = r23_02 / math.sin(theta4)
            sin_theta5 = self._clamp(sin_theta5, -1.0, 1.0)
            theta5 = math.atan2(sin_theta5, r13_02)
            theta6 = math.atan2(-R3_6[1][1], R3_6[1][0])

        cfg = configuration([theta1, theta2, theta3, theta4, theta5, theta6])
        if not self._within_limits(cfg.get_configuration()):
            return []
        return [cfg]

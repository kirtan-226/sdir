from ...postypes.SixDPos import SixDPos
from ...postypes.configuration import configuration
import math


class FwKinematics:
    _ALPHA = (0.0, -math.pi / 2, 0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, 0.0)
    _A = (0.0, 0.35, 1.25, -0.054, 0.0, 0.0, 0.0)
    _D = (0.75, 0.0, 0.0, 1.5, 0.0, 0.0, 0.303)
    _THETA_OFFSET = (0.0, -math.pi / 2, 0.0, 0.0, 0.0, 0.0)
    _IDENTITY = (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )

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
    def _rotation_to_euler(rot):
        r00, r01, r02 = rot[0][:3]
        r10, r11, r12 = rot[1][:3]
        r20, r21, r22 = rot[2][:3]
        sy = math.hypot(r00, r10)
        if sy > 1e-9:
            alpha = math.atan2(r21, r22)
            beta = math.atan2(-r20, sy)
            gamma = math.atan2(r10, r00)
        else:
            alpha = math.atan2(-r12, r11)
            beta = math.atan2(-r20, sy)
            gamma = 0.0
        return alpha, beta, gamma

    def get_fw_kinematics(self, config: configuration) -> SixDPos:
        joints = config.get_configuration()
        T = self._IDENTITY
        for i in range(6):
            theta = joints[i] + self._THETA_OFFSET[i]
            T = self._matmul(T, self._dh_transform(self._ALPHA[i], self._A[i], self._D[i], theta))
        T = self._matmul(T, self._dh_transform(self._ALPHA[6], self._A[6], self._D[6], 0.0))

        px, py, pz = T[0][3], T[1][3], T[2][3]
        alpha, beta, gamma = self._rotation_to_euler(T)
        return SixDPos(px, py, pz, alpha, beta, gamma)

from ...postypes.SixDPos import SixDPos
from ...postypes.configuration import configuration
import math


class FwKinematics:
    def get_fw_kinematics(self, config: configuration) -> SixDPos:
        # TODO: Implement the direct kinematics
        # Implement your logic here to compute the forward kinematics and derive position and euler angles.
        # Consider your units of rotation (degrees or radians) and translation (meters or millimeters).

        # For demonstration purposes, a fixed SixDPos is returned. Replace this with your computation.
        return SixDPos(1.757, 0.0, 1.91, 0, math.pi, 0)
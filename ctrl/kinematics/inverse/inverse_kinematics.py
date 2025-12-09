from ...postypes.configuration import configuration
import math


class InvKinematics:
    def get_inv_kinematics(self, _pos):
        # TODO: Implement inverse kinematics

        # For demonstration purposes, fixed Configurations are returned. Replace this with your computation.
        solutions = []
        solutions.append(configuration([0, 0, 1, 0, 0, 0]))
        solutions.append(configuration([1/8. * math.pi, 0, 1, 0, 0, 0]))
        solutions.append(configuration([2/8. * math.pi, 0, 1, 0, 0, 0]))
        solutions.append(configuration([3/8. * math.pi, 0, 1, 0, 0, 0]))
        solutions.append(configuration([4/8. * math.pi, 0, 1, 0, 0, 0]))
        solutions.append(configuration([5/8. * math.pi, 0, 1, 0, 0, 0]))
        solutions.append(configuration([6/8. * math.pi, 0, 1, 0, 0, 0]))
        solutions.append(configuration([7/8. * math.pi, 0, 1, 0, 0, 0]))

        return solutions
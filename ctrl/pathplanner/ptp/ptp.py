import math

from ...postypes.trajectory import trajectory
from ...postypes.configuration import configuration
from ...robot_specs import JOINT_LIMITS_RAD


class Ptp:
    _JOINT_STEP = 0.01  # radians
    _EPS = 1e-9

    @staticmethod
    def _assert_within_limits(cfg: configuration):
        joints = cfg.get_configuration()
        if len(joints) != len(JOINT_LIMITS_RAD):
            raise ValueError("Configuration length does not match joint limits")
        for idx, (value, (j_min, j_max)) in enumerate(zip(joints, JOINT_LIMITS_RAD)):
            if value < j_min - Ptp._EPS or value > j_max + Ptp._EPS:
                raise ValueError(f"Joint {idx + 1} value {value} out of limits [{j_min}, {j_max}]")

    def get_ptp_trajectory(self, start_cfg: configuration, end_cfg: configuration) -> trajectory:
        if not isinstance(start_cfg, configuration) or not isinstance(end_cfg, configuration):
            raise TypeError("Start and end inputs must be configuration instances")

        self._assert_within_limits(start_cfg)
        self._assert_within_limits(end_cfg)

        start_joints = tuple(start_cfg.get_configuration())
        end_joints = tuple(end_cfg.get_configuration())
        deltas = [end_joints[i] - start_joints[i] for i in range(len(start_joints))]
        max_delta = max(abs(delta) for delta in deltas)

        result = trajectory()
        if max_delta < self._EPS:
            result.add_configuration(configuration(list(start_joints)))
            return result

        steps = max(1, math.ceil(max_delta / self._JOINT_STEP))
        for step in range(steps + 1):
            if step == 0:
                joint_values = list(start_joints)
            elif step == steps:
                joint_values = list(end_joints)
            else:
                ratio = step / steps
                joint_values = [
                    start_joints[i] + ratio * deltas[i] for i in range(len(deltas))
                ]
            result.add_configuration(configuration(joint_values))

        return result

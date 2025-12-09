import math
from ...postypes.trajectory import trajectory
from ...postypes.configuration import configuration
from ...kinematics.direct.fw_kinematics import FwKinematics
from ...kinematics.inverse.inverse_kinematics import InvKinematics
from ...robot_specs import JOINT_LIMITS_RAD


class Lin:
    _LINEAR_STEP = 0.01  # meters
    _ANGULAR_STEP = 0.02  # radians
    _EPS = 1e-9

    def __init__(self):
        self._fw = FwKinematics()
        self._inv = InvKinematics()

    def _segment_count(self, delta):
        translational = math.sqrt(delta[0] ** 2 + delta[1] ** 2 + delta[2] ** 2)
        orientation_span = max(abs(delta[3]), abs(delta[4]), abs(delta[5]))
        trans_segments = math.ceil(translational / self._LINEAR_STEP) if translational > self._EPS else 0
        orient_segments = math.ceil(orientation_span / self._ANGULAR_STEP) if orientation_span > self._EPS else 0
        return max(1, trans_segments, orient_segments)

    @staticmethod
    def _assert_within_limits(cfg: configuration):
        joints = cfg.get_configuration()
        if len(joints) != len(JOINT_LIMITS_RAD):
            raise ValueError("Configuration length does not match joint limits")
        for idx, (value, (j_min, j_max)) in enumerate(zip(joints, JOINT_LIMITS_RAD)):
            if value < j_min - Lin._EPS or value > j_max + Lin._EPS:
                raise ValueError(f"Joint {idx + 1} value {value} out of limits [{j_min}, {j_max}]")

    def get_lin_trajectory(self, _start_cfg, _end_cfg):
        if not isinstance(_start_cfg, configuration) or not isinstance(_end_cfg, configuration):
            raise TypeError("Start and end inputs must be configuration instances")

        self._assert_within_limits(_start_cfg)
        self._assert_within_limits(_end_cfg)

        trajectory_ins = trajectory()
        start_pose = self._fw.get_fw_kinematics(_start_cfg).get_position()
        end_pose = self._fw.get_fw_kinematics(_end_cfg).get_position()
        delta_pose = [end_pose[i] - start_pose[i] for i in range(6)]
        max_delta = max(abs(value) for value in delta_pose)

        start_joints = list(_start_cfg.get_configuration())
        end_joints = list(_end_cfg.get_configuration())

        if max_delta < self._EPS:
            trajectory_ins.add_configuration(configuration(start_joints))
            return trajectory_ins

        segments = self._segment_count(delta_pose)
        for step in range(segments + 1):
            if step == 0:
                cfg = configuration(start_joints)
            elif step == segments:
                cfg = configuration(end_joints)
            else:
                ratio = step / segments
                interpolated_pose = tuple(start_pose[i] + ratio * delta_pose[i] for i in range(6))
                ik_solution = self._inv.get_inv_kinematics(interpolated_pose)
                if not ik_solution:
                    raise ValueError("Inverse kinematics failed for interpolated pose")
                cfg = ik_solution[0]
                self._assert_within_limits(cfg)
            trajectory_ins.add_configuration(cfg)

        return trajectory_ins

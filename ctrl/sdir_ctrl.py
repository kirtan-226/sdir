from typing import List
from .postypes.configuration import configuration
from .postypes.trajectory import trajectory
from .postypes.SixDPos import SixDPos
from .kinematics.direct.fw_kinematics import FwKinematics
from .kinematics.inverse.inverse_kinematics import InvKinematics
from .pathplanner.ptp.ptp import Ptp
from .pathplanner.lin.lin import Lin


class SdirCtrl:
    def get_pos_from_config(self, config: configuration) -> SixDPos:
        fw_kinematics = FwKinematics()
        new_pos = fw_kinematics.get_fw_kinematics(config)
        return new_pos

    def get_config_from_pos(self, pos: SixDPos) -> List[configuration]:
        inv_kinematics = InvKinematics()
        new_cfg = inv_kinematics.get_inv_kinematics(pos)
        return new_cfg

    def move_robot_ptp(self, start, end) -> trajectory:
        if isinstance(start, configuration) and isinstance(end, configuration):
            ptp = Ptp()
            return ptp.get_ptp_trajectory(start, end)
        else:
            raise ValueError("Invalid argument types for move_robot_ptp")

    def move_robot_lin(self, start, end) -> trajectory:
        if isinstance(start, configuration) and isinstance(end, configuration):
            lin = Lin()
            return lin.get_lin_trajectory(start, end)
        else:
            raise ValueError("Invalid argument types for move_robot_lin")
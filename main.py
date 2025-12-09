import os
import time
from ctrl.sdir_ctrl import SdirCtrl
from ctrl.com.json_handler import JsonHandler
from ctrl.postypes.SixDPos import SixDPos
from ctrl.postypes.configuration import configuration
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


# Define operation modes
class OpMode:
    CFG_2_POS = 0
    POS_2_CFG = 1
    PTP = 2
    PTPSYNC = 3
    LIN = 4


def initial():
    global client, sim, jh
    portNb = 23000 # default value

    try:
        # Initialize the CoppeliaSim remote API client
        client = RemoteAPIClient('127.0.0.1', portNb)
        sim = client.require('sim')
        scene_path = os.path.abspath('sim/sdir.ttt')
        print("Connected to CoppeliaSim")

        # Load the scene
        sim.loadScene(scene_path)

        # Get object handles for the robot joints
        jh = [0] * 6
        for i in range(6):
            Joint = '/KR120_2700_2_joint' + str(i + 1)
            jh[i] = sim.getObject(Joint)

        # Start the simulation
        sim.startSimulation()
        print("Simulation Started")

    except Exception as e:
        print(f"Error during initialization: {str(e)}")
        raise e

    return client


def main():
    global client, sim
    print("This is the entry point of the SDIR programming project")
    ctrl = SdirCtrl()  # Create an instance of SdirCtrl
    c = [0.0] * 6

    try:
        initial()  # Connect to CoppeliaSim
    except Exception as e:
        print(f"Error during initialization: {str(e)}, exiting.")
        return

    running = True
    while running:  # Use the flag in the loop condition
        signal_value = sim.getStringSignal("callsignal")

        if signal_value is not None and len(signal_value) > 0:
            json_handler = JsonHandler(signal_value)
            mode = json_handler.get_op_mode()

            if mode == OpMode.POS_2_CFG:
                pos = SixDPos(json_handler.get_data()[0])
                result_cfg = ctrl.get_config_from_pos(pos)
                json_return_string = json_handler.get_json_string(result_cfg)
                sim.setStringSignal("returnsignal", json_return_string)
                scriptHandle = sim.getScriptHandle('Coord_Dialog')
                sim.callScriptFunction("returnSignal", scriptHandle)

            if mode == OpMode.CFG_2_POS:
                cfg = configuration(json_handler.get_data()[0])
                return_pos = ctrl.get_pos_from_config(cfg)
                json_return_string = json_handler.get_json_string(return_pos)
                sim.setStringSignal("returnsignal", json_return_string)
                scriptHandle = sim.getScriptHandle('Coord_Dialog')
                sim.callScriptFunction("returnSignal", scriptHandle)

            if mode == OpMode.PTP:
                start_cfg = configuration(json_handler.get_data()[0])
                end_cfg = configuration(json_handler.get_data()[1])
                trajectory = ctrl.move_robot_ptp(start_cfg, end_cfg)
                for cur_cfg in trajectory.get_all_configuration():
                    c[0] = float(cur_cfg[0])
                    c[1] = float(cur_cfg[1])
                    c[2] = float(cur_cfg[2])
                    c[3] = float(cur_cfg[3])
                    c[4] = float(cur_cfg[4])
                    c[5] = float(cur_cfg[5])
                    script_handle = sim.getScriptHandle('KR120_2700_2')
                    sim.callScriptFunction("runConfig", script_handle, len(c), c, "", "")
                    # Synchronize with simulation environment
                    time.sleep(0.05)

            if mode == OpMode.PTPSYNC:
                start_cfg = configuration(json_handler.get_data()[0])
                end_cfg = configuration(json_handler.get_data()[1])
                trajectory = ctrl.move_robot_ptp(start_cfg, end_cfg)
                for cur_cfg in trajectory.get_all_configuration():
                    c[0] = float(cur_cfg[0])
                    c[1] = float(cur_cfg[1])
                    c[2] = float(cur_cfg[2])
                    c[3] = float(cur_cfg[3])
                    c[4] = float(cur_cfg[4])
                    c[5] = float(cur_cfg[5])
                    script_handle = sim.getScriptHandle('KR120_2700_2')
                    sim.callScriptFunction("runConfig", script_handle, len(c), c, "", "")
                    # Synchronize with simulation environment
                    time.sleep(0.05)

            if mode == OpMode.LIN:
                start_cfg = configuration(json_handler.get_data()[0])
                end_cfg = configuration(json_handler.get_data()[1])
                trajectory = ctrl.move_robot_lin(start_cfg, end_cfg)
                for cur_cfg in trajectory.get_all_configuration():
                    c[0] = float(cur_cfg[0])
                    c[1] = float(cur_cfg[1])
                    c[2] = float(cur_cfg[2])
                    c[3] = float(cur_cfg[3])
                    c[4] = float(cur_cfg[4])
                    c[5] = float(cur_cfg[5])
                    script_handle = sim.getScriptHandle('KR120_2700_2')
                    sim.callScriptFunction("runConfig", script_handle, len(c), c, "", "")
                    # Synchronize with simulation environment
                    time.sleep(0.05)

            sim.clearStringSignal("callsignal")
            time.sleep(0.05)  # Add a delay for synchronization


if __name__ == "__main__":
    main()
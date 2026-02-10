# terminal_monitor.py
# python3 pyAgxArm/demo/detect_arm.py --can_port can0 --hz 10 --req_flag 0
# python3 pyAgxArm/demo/detect_arm.py --can_port can0 --hz 10 --req_flag 1
import time
import argparse
from enum import Enum
import os
import sys
import threading
from pyAgxArm.api.agx_arm_factory import AgxArmFactory, create_agx_arm_config
from pyAgxArm.protocols.can_protocol.drivers.piper.default.driver import Driver
# Windows 和 Unix 不同的键盘输入方式
try:
    import termios
    import tty
except ImportError:
    import msvcrt

parser = argparse.ArgumentParser(description="Piper Terminal Table Monitor")
parser.add_argument("--robot", type=str, default="piper", help="robotic arm type")
parser.add_argument("--can_port", type=str, default="can0", help="CAN port name")
parser.add_argument("--hz", type=float, default=10, help="Refresh rate (Hz), range: 0.5 ~ 200")
args = parser.parse_args()

exit_flag = False

robot_cfg = create_agx_arm_config(robot=args.robot, comm="can", channel=args.can_port, interface="socketcan")
robot: Driver = AgxArmFactory.create_arm(robot_cfg)
robot.connect()

effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)

def clamp_refresh_rate(rate_hz):
    return max(0.5, min(rate_hz, 200.0))

def clear_terminal():
    os.system("cls" if os.name == "nt" else "clear")

def key_listener():
    global exit_flag
    if os.name == 'nt':
        while True:
            if msvcrt.kbhit():
                if msvcrt.getch().lower() == b'q':
                    exit_flag = True
                    print("exit...")
                    break
    else:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while True:
                if sys.stdin.read(1).lower() == 'q':
                    exit_flag = True
                    print("exit...")
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

class ArmStatusTool():
    class CtrlMode(Enum):
        Standby = 0x00
        Can_ctrl = 0x01
        Teaching_mode = 0x02
        Ethernet_control_mode = 0x03
        WiFi_control_mode = 0x04
        Remote_control_mode = 0x05
        Linkage_teaching_input_mode = 0x06
        Offline_trajectory_mode = 0x07
        TCP_control_mode = 0x08
        def __str__(self):
            return f"{self.name} (0x{self.value:X})"
        def __repr__(self):
            return f"{self.name}: 0x{self.value:X}"
        @classmethod
        def from_value(cls, val):
            if isinstance(val, str):
                val = int(val, 0)  # 自动识别 '0x02'、'02'、'2' 等形式
            return cls(val)
    class ArmStatus(Enum):
        Normal = 0x00
        Emergency_stop = 0x01
        No_solution = 0x02
        Singularity_point = 0x03
        Target_angle_exceeds_limit = 0x04
        Joint_communication_err = 0x05
        Joint_brake_not_released = 0x06
        Collision_occurred = 0x07
        Overspeed_during_teaching_drag = 0x08
        Joint_status_err = 0x09
        Other_err = 0x0A
        Teaching_record = 0x0B
        Teaching_execution = 0x0C
        Teaching_pause = 0x0D
        Main_controller_NTC_over_temperature = 0x0E
        Release_resistor_NTC_over_temperature = 0x0F
        def __str__(self):
            return f"{self.name} (0x{self.value:X})"
        def __repr__(self):
            return f"{self.name}: 0x{self.value:X}"
        @classmethod
        def from_value(cls, val):
            if isinstance(val, str):
                val = int(val, 0)  # 自动识别 '0x02'、'02'、'2' 等形式
            return cls(val)
    class ModeFeedback(Enum):
        MOVE_P = 0x00
        MOVE_J = 0x01
        MOVE_L = 0x02
        MOVE_C = 0x03
        MOVE_MIT = 0x04
        MOVE_CPV = 0x05
        def __str__(self):
            return f"{self.name} (0x{self.value:X})"
        def __repr__(self):
            return f"{self.name}: 0x{self.value:X}"
        @classmethod
        def from_value(cls, val):
            if isinstance(val, str):
                val = int(val, 0)  # 自动识别 '0x02'、'02'、'2' 等形式
            return cls(val)
    class MotionStatus(Enum):
        Reached_the_target_position = 0x00
        Not_yet_reached_the_target_position = 0x01
        def __str__(self):
            return f"{self.name} (0x{self.value:X})"
        def __repr__(self):
            return f"{self.name}: 0x{self.value:X}"
        @classmethod
        def from_value(cls, val):
            if isinstance(val, str):
                val = int(val, 0)  # 自动识别 '0x02'、'02'、'2' 等形式
            return cls(val)
    # def __str__(self):
    #     return f"{self.name} (0x{self.value:X})"
    # def __repr__(self):
    #     return f"{self.name}: 0x{self.value:X}"

def display_table(refresh_interval):
    global exit_flag
    global args
    listener_thread = threading.Thread(target=key_listener, daemon=True)
    listener_thread.start()
    start = 0
    while not exit_flag:  

        try:
            if time.time() - start > 5.0:
                firmware = robot.get_firmware()
                if firmware is None:
                    continue

                joint_angle_vel_limits = [robot.get_joint_angle_vel_limits(i) for i in range(1, robot.joint_nums + 1)]
                if None in joint_angle_vel_limits:
                    continue

                joint_acc_limits = [robot.get_joint_acc_limits(i) for i in range(1, robot.joint_nums + 1)]
                if None in joint_acc_limits:
                    continue

                flange_vel_acc = robot.get_flange_vel_acc_limits()
                if flange_vel_acc is None:
                    continue

                crash_protection = robot.get_crash_protection_rating()
                if crash_protection is None:
                    continue

                gripper_teaching_pendant_param = effector.get_gripper_teaching_pendant_param()
                if gripper_teaching_pendant_param is None:
                    continue
                
                start = time.time()

            robot_status = robot.get_arm_status()
            if robot_status is None:
                continue

            joint = robot.get_joint_angles()
            if joint is None:
                continue

            flange_pose = robot.get_flange_pose()
            if flange_pose is None:
                continue

            motor_states = [robot.get_motor_states(i) for i in range(1, robot.joint_nums + 1)]
            if None in motor_states:
                continue

            driver_states = [robot.get_driver_states(i) for i in range(1, robot.joint_nums + 1)]
            if None in driver_states:
                continue

            gripper_status = effector.get_gripper_status()
            if gripper_status is None:
                continue

        except Exception as e:
            print(f"Error: {e}")
            # continue
            exit()

        clear_terminal()
        print(time.strftime("%a %b %d %H:%M:%S %Y"))
        # print(f"+{'-'*87}+")
        print(f"+{'='*107}+")
        print(f"Software Ver : {firmware['software_version']:<10}"
              f"\n"
              f"Hardware Ver : {firmware['hardware_version']:<10}"
              f"\n"
              f"Producttion Date: {firmware['production_date']:<10}"
            #   f"\n"
            #   f"CAN PORT     : {can_port:<15}  SDK Ver: {piper.GetCurrentSDKVersion().value:<11}"
            #   f"\n"
            #   f"Interface Ver: {piper.GetCurrentInterfaceVersion().value:<15}  Protocol Ver: {piper.GetCurrentProtocolVersion().value:<15}"
              )

        print(f"+{'-'*107}+\n"
              f"{'ArmStatus'} :\n"
              f"{'ctrl_mode':<15}{robot_status.msg.ctrl_mode}\n"
              f"{'arm_status':<15}{robot_status.msg.arm_status}\n"
              f"{'mode_feedback':<15}{robot_status.msg.mode_feedback}\n"
              f"{'motion_status':<15}{robot_status.msg.motion_status}"
              )
        
        print(f"+{'-'*107}+\n"
              f"|{'JointState':<16}|{'J1':^15}{'J2':^15}{'J3':^15}{'J4':^15}{'J5':^15}{'J6':^15}|\n"
              f"+{'-'*16:^16}+{'-'*90:^}+\n"
              f"|{'position(rad)':<16}|"
              f"{round(joint.msg[0], 3):^15}"
              f"{round(joint.msg[1], 3):^15}"
              f"{round(joint.msg[2], 3):^15}"
              f"{round(joint.msg[3], 3):^15}"
              f"{round(joint.msg[4], 3):^15}"
              f"{round(joint.msg[5], 3):^15}|\n"
              f"|{'position(rad)':<16}|"
              f"{round(motor_states[0].msg.pos, 3):^15}"
              f"{round(motor_states[1].msg.pos, 3):^15}"
              f"{round(motor_states[2].msg.pos, 3):^15}"
              f"{round(motor_states[3].msg.pos, 3):^15}"
              f"{round(motor_states[4].msg.pos, 3):^15}"
              f"{round(motor_states[5].msg.pos, 3):^15}|\n"
              f"|{'cur_spd(rad/s)':<16}|"
              f"{round(motor_states[0].msg.motor_speed, 3):^15}"
              f"{round(motor_states[1].msg.motor_speed, 3):^15}"
              f"{round(motor_states[2].msg.motor_speed, 3):^15}"
              f"{round(motor_states[3].msg.motor_speed, 3):^15}"
              f"{round(motor_states[4].msg.motor_speed, 3):^15}"
              f"{round(motor_states[5].msg.motor_speed, 3):^15}|\n"
              f"|{'current(A)':<16}|"
              f"{round(motor_states[0].msg.current, 3):^15}"
              f"{round(motor_states[1].msg.current, 3):^15}"
              f"{round(motor_states[2].msg.current, 3):^15}"
              f"{round(motor_states[3].msg.current, 3):^15}"
              f"{round(motor_states[4].msg.current, 3):^15}"
              f"{round(motor_states[5].msg.current, 3):^15}|\n"
              f"|{'torque(N.m)':<16}|"
              f"{round(motor_states[0].msg.torque, 3):^15}"
              f"{round(motor_states[1].msg.torque, 3):^15}"
              f"{round(motor_states[2].msg.torque, 3):^15}"
              f"{round(motor_states[3].msg.torque, 3):^15}"
              f"{round(motor_states[4].msg.torque, 3):^15}"
              f"{round(motor_states[5].msg.torque, 3):^15}|\n"
              f"|{'voltage(V)':<16}|"
              f"{round(driver_states[0].msg.vol, 1):^15}"
              f"{round(driver_states[1].msg.vol, 1):^15}"
              f"{round(driver_states[2].msg.vol, 1):^15}"
              f"{round(driver_states[3].msg.vol, 1):^15}"
              f"{round(driver_states[4].msg.vol, 1):^15}"
              f"{round(driver_states[5].msg.vol, 1):^15}|\n"
              f"|{'foc_temp(°C)':<16}|"
              f"{round(driver_states[0].msg.foc_temp):^15}"
              f"{round(driver_states[1].msg.foc_temp):^15}"
              f"{round(driver_states[2].msg.foc_temp):^15}"
              f"{round(driver_states[3].msg.foc_temp):^15}"
              f"{round(driver_states[4].msg.foc_temp):^15}"
              f"{round(driver_states[5].msg.foc_temp):^15}|\n"
              f"|{'motor_temp(°C)':<16}|"
              f"{round(driver_states[0].msg.motor_temp):^15}"
              f"{round(driver_states[1].msg.motor_temp):^15}"
              f"{round(driver_states[2].msg.motor_temp):^15}"
              f"{round(driver_states[3].msg.motor_temp):^15}"
              f"{round(driver_states[4].msg.motor_temp):^15}"
              f"{round(driver_states[5].msg.motor_temp):^15}|\n"
              f"|{'max_spd(rad/s)':<16}|"
              f"{round(joint_angle_vel_limits[0].msg.max_joint_spd, 3):^15}"
              f"{round(joint_angle_vel_limits[1].msg.max_joint_spd, 3):^15}"
              f"{round(joint_angle_vel_limits[2].msg.max_joint_spd, 3):^15}"
              f"{round(joint_angle_vel_limits[3].msg.max_joint_spd, 3):^15}"
              f"{round(joint_angle_vel_limits[4].msg.max_joint_spd, 3):^15}"
              f"{round(joint_angle_vel_limits[5].msg.max_joint_spd, 3):^15}|\n"
              f"|{'max_acc(rad/s^2)':<16}|"
              f"{round(joint_acc_limits[0].msg.max_joint_acc, 3):^15}"
              f"{round(joint_acc_limits[1].msg.max_joint_acc, 3):^15}"
              f"{round(joint_acc_limits[2].msg.max_joint_acc, 3):^15}"
              f"{round(joint_acc_limits[3].msg.max_joint_acc, 3):^15}"
              f"{round(joint_acc_limits[4].msg.max_joint_acc, 3):^15}"
              f"{round(joint_acc_limits[5].msg.max_joint_acc, 3):^15}|\n"
              f"|{'collision_level':<16}|"
              f"{round(crash_protection.msg[0]):^15}"
              f"{round(crash_protection.msg[1]):^15}"
              f"{round(crash_protection.msg[2]):^15}"
              f"{round(crash_protection.msg[3]):^15}"
              f"{round(crash_protection.msg[4]):^15}"
              f"{round(crash_protection.msg[5]):^15}|\n"
              f"|{'angle_limit(rad)':<16}|"
              f"[{round(joint_angle_vel_limits[0].msg.min_angle_limit,3):<6},"
              f"{round(joint_angle_vel_limits[0].msg.max_angle_limit, 3):<6}]"
              f"[{round(joint_angle_vel_limits[1].msg.min_angle_limit,3):<6},"
              f"{round(joint_angle_vel_limits[1].msg.max_angle_limit, 3):<6}]"
              f"[{round(joint_angle_vel_limits[2].msg.min_angle_limit,3):<6},"
              f"{round(joint_angle_vel_limits[2].msg.max_angle_limit, 3):<6}]"
              f"[{round(joint_angle_vel_limits[3].msg.min_angle_limit,3):<6},"
              f"{round(joint_angle_vel_limits[3].msg.max_angle_limit, 3):<6}]"
              f"[{round(joint_angle_vel_limits[4].msg.min_angle_limit,3):<6},"
              f"{round(joint_angle_vel_limits[4].msg.max_angle_limit, 3):<6}]"
              f"[{round(joint_angle_vel_limits[5].msg.min_angle_limit,3):<6},"
              f"{round(joint_angle_vel_limits[5].msg.max_angle_limit, 3):<6}]|\n"
              f"|{'status----------':<16}|{'-'*90:^}|\n"
              f"|{'low_vol_err':<16}|"
              f"{str(driver_states[0].msg.foc_status.voltage_too_low):^15}"
              f"{str(driver_states[1].msg.foc_status.voltage_too_low):^15}"
              f"{str(driver_states[2].msg.foc_status.voltage_too_low):^15}"
              f"{str(driver_states[3].msg.foc_status.voltage_too_low):^15}"
              f"{str(driver_states[4].msg.foc_status.voltage_too_low):^15}"
              f"{str(driver_states[5].msg.foc_status.voltage_too_low):^15}|\n"
              f"|{'motor_overheat':<16}|"
              f"{str(driver_states[0].msg.foc_status.motor_overheating):^15}"
              f"{str(driver_states[1].msg.foc_status.motor_overheating):^15}"
              f"{str(driver_states[2].msg.foc_status.motor_overheating):^15}"
              f"{str(driver_states[3].msg.foc_status.motor_overheating):^15}"
              f"{str(driver_states[4].msg.foc_status.motor_overheating):^15}"
              f"{str(driver_states[5].msg.foc_status.motor_overheating):^15}|\n"
              f"|{'foc_overcurrent':<16}|"
              f"{str(driver_states[0].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[1].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[2].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[3].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[4].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[5].msg.foc_status.driver_overcurrent):^15}|\n"
              f"|{'foc_overheat':<16}|"
              f"{str(driver_states[0].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[1].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[2].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[3].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[4].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[5].msg.foc_status.driver_overcurrent):^15}|\n"
              f"|{'collision_status':<16}|"
              f"{str(driver_states[0].msg.foc_status.collision_status):^15}"
              f"{str(driver_states[1].msg.foc_status.collision_status):^15}"
              f"{str(driver_states[2].msg.foc_status.collision_status):^15}"
              f"{str(driver_states[3].msg.foc_status.collision_status):^15}"
              f"{str(driver_states[4].msg.foc_status.collision_status):^15}"
              f"{str(driver_states[5].msg.foc_status.collision_status):^15}|\n"
              f"|{'foc_err':<16}|"
              f"{str(driver_states[0].msg.foc_status.driver_error_status):^15}"
              f"{str(driver_states[1].msg.foc_status.driver_error_status):^15}"
              f"{str(driver_states[2].msg.foc_status.driver_error_status):^15}"
              f"{str(driver_states[3].msg.foc_status.driver_error_status):^15}"
              f"{str(driver_states[4].msg.foc_status.driver_error_status):^15}"
              f"{str(driver_states[5].msg.foc_status.driver_error_status):^15}|\n"
              f"|{'enable_status':<16}|"
              f"{str(driver_states[0].msg.foc_status.driver_enable_status):^15}"
              f"{str(driver_states[1].msg.foc_status.driver_enable_status):^15}"
              f"{str(driver_states[2].msg.foc_status.driver_enable_status):^15}"
              f"{str(driver_states[3].msg.foc_status.driver_enable_status):^15}"
              f"{str(driver_states[4].msg.foc_status.driver_enable_status):^15}"
              f"{str(driver_states[5].msg.foc_status.driver_enable_status):^15}|\n"
              f"|{'stall_protection':<16}|"
              f"{str(driver_states[0].msg.foc_status.stall_status):^15}"
              f"{str(driver_states[1].msg.foc_status.stall_status):^15}"
              f"{str(driver_states[2].msg.foc_status.stall_status):^15}"
              f"{str(driver_states[3].msg.foc_status.stall_status):^15}"
              f"{str(driver_states[4].msg.foc_status.stall_status):^15}"
              f"{str(driver_states[5].msg.foc_status.stall_status):^15}|\n"
              f"|{'commuciation_err':<16}|"
              f"{str(robot_status.msg.err_status.communication_status_joint_1):^15}"
              f"{str(robot_status.msg.err_status.communication_status_joint_2):^15}"
              f"{str(robot_status.msg.err_status.communication_status_joint_3):^15}"
              f"{str(robot_status.msg.err_status.communication_status_joint_4):^15}"
              f"{str(robot_status.msg.err_status.communication_status_joint_5):^15}"
              f"{str(robot_status.msg.err_status.communication_status_joint_6):^15}|\n"
              f"|{'over_angle':<16}|"
              f"{str(robot_status.msg.err_status.joint_1_angle_limit):^15}"
              f"{str(robot_status.msg.err_status.joint_2_angle_limit):^15}"
              f"{str(robot_status.msg.err_status.joint_3_angle_limit):^15}"
              f"{str(robot_status.msg.err_status.joint_4_angle_limit):^15}"
              f"{str(robot_status.msg.err_status.joint_5_angle_limit):^15}"
              f"{str(robot_status.msg.err_status.joint_6_angle_limit):^15}|\n"
              f"+{'-'*107}+"
              )
        print(
              f"{'Flange Pose(Euler):':<108}|"
              f"\n"
              f"{'xyz(m)':<12}"
              f"{round(flange_pose.msg[0], 3):<9}"
              f"{round(flange_pose.msg[1], 3):<9}"
              f"{round(flange_pose.msg[2], 3):<9}|"
              f"{'max_linear_vel':^20}"
              f"{round(flange_vel_acc.msg.end_max_linear_vel, 3):^7}"
              f"{'m/s':<5}|"
              f"{'max_angular_vel':^20}"
              f"{round(flange_vel_acc.msg.end_max_angular_vel, 3):^7}"
              f"{'rad/s':<8}|"
              f"\n"
              f"{'rpy(rad)':<12}"
              f"{round(flange_pose.msg[3], 3):<9}"
              f"{round(flange_pose.msg[4], 3):<9}"
              f"{round(flange_pose.msg[5], 3):<9}|"
              f"{'max_linear_acc':^20}"
              f"{round(flange_vel_acc.msg.end_max_linear_acc, 3):^7}"
              f"{'m/s^2':<5}|"
              f"{'max_angular_acc':^20}"
              f"{round(flange_vel_acc.msg.end_max_angular_acc, 3):^7}"
              f"{'rad/s^2':<8}|"
              )
        print(f"+{'-'*107}+\n"
              f"{'Gripper&Teaching':<108}|"
              
              f"\n"
              f"{'gripper_pos(m)':<21}{round(gripper_status.msg.width, 3):<6}|"
              f"{'Status code :':<59}"
              f"{'|':>22}"
              f"\n"
              f"{'gripper_force(N.m)':<21}{round(gripper_status.msg.force, 3):<6}|"
              f"{'voltage_too_low':<23}{str(gripper_status.msg.foc_status.voltage_too_low):<6}|"
              f"{'motor_overheating':<23}{str(gripper_status.msg.foc_status.motor_overheating):<6}"
              f"{'|':>22}"
              f"\n"
              f"{'teaching_per':<21}{gripper_teaching_pendant_param.msg.teaching_range_per:<6}|"
              f"{'driver_overcurrent':<23}{str(gripper_status.msg.foc_status.driver_overcurrent):<6}|"
              f"{'driver_overheating':<23}{str(gripper_status.msg.foc_status.driver_overheating):<6}"
              f"{'|':>22}"
              f"\n"
              f"{'max_range_config(m)':<21}{gripper_teaching_pendant_param.msg.max_range_config:<6}|"
              f"{'sensor_status':<23}{str(gripper_status.msg.foc_status.sensor_status):<6}|"
              f"{'driver_error_status':<23}{str(gripper_status.msg.foc_status.driver_error_status):<6}"
                f"{'|':>22}"              
              f"\n"
              f"{'teaching_friction':<21}{gripper_teaching_pendant_param.msg.teaching_friction:<6}|"
              f"{'driver_enable_status':<23}{str(gripper_status.msg.foc_status.driver_enable_status):<6}|"
              f"{'homing_status':<23}{str(gripper_status.msg.foc_status.homing_status):<6}"
              f"{'|':>22}"
              )
        # 单独打印 FPS 类
        print(f"+{'='*52}FPS{'='*52}+\n"
              f"{'All FPS':<15}: {round(robot.get_fps() + effector.get_fps())}\n"
              f"{'Arm Status':<15}: {round(robot_status.hz):<5}  {'Flange Pose':<15}: {round(flange_pose.hz):<5}\n"
              f"{'Joint Msg':<15}: {round(joint.hz):<5}  {'Gripper Msg':<15}: {round(gripper_status.hz):<5}\n"
              f"{'High Spd Info':<15}: {round(motor_states[0].hz):<5}  {'Low Spd Info':<15}: {round(driver_states[0].hz):<5}")
        print("=" * 109)
        print("Press 'q' to quit")
        time.sleep(refresh_interval)

def main():
    hz = clamp_refresh_rate(args.hz)
    refresh_interval = 1.0 / hz
    display_table(refresh_interval)

if __name__ == "__main__":
    main()

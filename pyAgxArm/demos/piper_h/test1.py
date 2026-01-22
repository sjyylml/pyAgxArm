import time
from pyAgxArm.api.agx_arm_factory import create_agx_arm_config, AgxArmFactory


def wait_motion_done(robot, timeout: float = 5.0, poll_interval: float = 0.1) -> bool:
    """Wait until `robot.get_arm_status().msg.motion_status == 0` or timeout."""
    time.sleep(0.1)
    start_t = time.monotonic()
    while True:
        status = robot.get_arm_status()
        if status is not None and getattr(status.msg, "motion_status", None) == 0:
            print("motion done")
            return True
        if time.monotonic() - start_t > timeout:
            print(f"wait motion done timeout ({timeout:.1f}s)")
            return False
        time.sleep(poll_interval)


robot_cfg = create_agx_arm_config(robot="piper_h", comm="can", channel="can0", interface="socketcan")
print(robot_cfg)
robot = AgxArmFactory.create_arm(robot_cfg)
robot.connect()
print(robot.get_channel())
print(robot.__doc__)

end_effector = robot.init_effector(robot.EFFECTOR.AGX_GRIPPER)
# end_effector = robot.init_effector(robot.EFFECTOR.REVO2)
print(end_effector.__doc__)


# -------------------------- Basic ---------------------------

# while not robot.enable():
#     time.sleep(0.01)

# while not robot.disable():
#     time.sleep(0.01)

# robot.set_speed_percent(100)
# robot.set_installation_pos(robot.INSTALLATION_POS.HORIZONTAL)
# robot.set_motion_mode(robot.MOTION_MODE.J)

# print(robot.set_links_vel_acc_period_feedback(False))


# -------------------------- Move ----------------------------

# robot.move_p([0.1, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])
# time.sleep(0.1)
# wait_motion_done(robot, timeout=5.0)

# robot.move_l([0.2, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])
# time.sleep(0.1)
# wait_motion_done(robot, timeout=5.0)

# start_pose = [0.2, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0]
# mid_pose = [0.2, 0.05, 0.35, 0.0, 1.570796326794896619, 0.0]
# end_pose = [0.2, 0.0, 0.4, 0.0, 1.570796326794896619, 0.0]
# robot.move_c(start_pose, mid_pose, end_pose)
# time.sleep(0.1)
# wait_motion_done(robot, timeout=5.0)

# robot.move_j([0.0, 0.4, -0.4, 0, -0.4, 0])
# time.sleep(0.1)
# wait_motion_done(robot, timeout=5.0)

# robot.move_j([0] * 6)
# time.sleep(0.1)
# wait_motion_done(robot, timeout=5.0)


# --------------------------  MIT mode ------------------------

# robot.move_js([0, 0.2, -0.2, 0, -0.2, 0])
# time.sleep(2)

# robot.move_mit(1, 0.0)
# robot.move_mit(2, 0.4)
# robot.move_mit(3, -0.4)
# robot.move_mit(4, 0.0)
# robot.move_mit(5, -0.4)
# robot.move_mit(6, 0.4)
# time.sleep(2)


# -------------------------- reset ----------------------------

# robot.move_js([0] * 6)
# time.sleep(2)
# robot.electronic_emergency_stop()
# time.sleep(1)

# robot.reset()
# time.sleep(1.5)
# robot.enable()
# robot.move_j([0.0] * 6)
# time.sleep(0.01)
# robot.disable()


# -------------------------- Get data -------------------------


while True:
    break

    # print(robot.get_fps())
    # print(robot.is_ok())
    # print(end_effector.get_fps())
    # print(end_effector.is_ok())

    t = time.time()

    # print(robot.get_arm_status())
    # print(end_effector.get_gripper_status())

    # print(robot.get_firmware())
    # print(robot.get_joint_states())
    # print(robot.get_flange_pose())

    # print(robot.get_joint_ctrl_states())
    # print(end_effector.get_gripper_ctrl_states())

    # print(robot.get_driver_states(1))
    # print(robot.get_motor_states(1))
    # print(robot.get_joint_enable_status(1))
    # print(robot.get_joints_enable_status_list())

    # print(robot.get_flange_vel_acc_limits())

    # print(robot.get_crash_protection_rating())

    # print(end_effector.get_gripper_teaching_pendant_param())

    # for driver_state in [robot.get_driver_states(i) for i in range(1, robot.joint_nums + 1)]:
    #     print(driver_state)
    #     print()

    # for motor_state in [robot.get_motor_states(i) for i in range(1, robot.joint_nums + 1)]:
    #     print(motor_state)
    #     print()

    # for j in [robot.get_joint_angle_vel_limits(i) for i in range(1, robot.joint_nums + 1)]:
    #     print(j)
    #     print()

    # for j in [robot.get_joint_acc_limits(i) for i in range(1, robot.joint_nums + 1)]:
    #     print(j)
    #     print()

    # Revo2
    # print(end_effector.get_hand_status())
    # print(end_effector.get_finger_pos())
    # print(end_effector.get_finger_spd())
    # print(end_effector.get_finger_current())

    t = round((time.time() - t) * 1000, 4)

    if t > 1:
        print("\nTime elapsed: ", t, "ms")

    print()

    time.sleep(0.005)


# -------------------------- Motor torque measurements -------

# robot.move_mit(6, 0.0, v_des=0.0, kp=0, kd=0, t_ff=0.8)

# import numpy as np

# data = []

# while True:
#     data.append(robot.get_motor_states(6).msg.torque)
#     time.sleep(0.02)
#     if len(data) > 100:
#         break

# data = np.mean(data)
# print(data)


# -------------------------- Gripper --------------------------

# Calibrate the gripper.
# 1. Disable the gripper.
# end_effector.disable_gripper()

# 2. Set the actual maximum stroke of the gripper.
# end_effector.set_gripper_teaching_pendant_param(max_range_config=0.07)  # 0.07m
# end_effector.set_gripper_teaching_pendant_param(max_range_config=0.1)   # 0.1m

# 3. Manually squeeze and close the gripper.

# 4. Run the calibration command.
# end_effector.calibrate_gripper()

# 5. Move the gripper.
# robot.set_motion_mode(robot.MOTION_MODE.P)
# end_effector.move_gripper(0.07)
# time.sleep(0.2)
# end_effector.move_gripper(0)


# -------------------------- Revo2 --------------------------

# end_effector.position_ctrl()
# end_effector.speed_ctrl()
# end_effector.current_ctrl()

# The tip of the thumb moves to position 100 in 2 seconds.
# end_effector.position_time_ctrl(mode='pos', thumb_tip=100)
# end_effector.position_time_ctrl(mode='time', thumb_tip=200)


# -------------------------- Master Arm --------------------------

# robot.set_master_mode()
# robot.set_slave_mode()
# time.sleep(2)
# robot.move_master_to_home()
# time.sleep(2)
# robot.restore_master_drag_mode()


# ------------------------- Other --------------------------------

# while not robot.disable():
#     time.sleep(0.01)

# print(robot.calibrate_joint(1))

# -----------------------------------------------------------------------------------------------

# print(robot.set_payload(robot.PAYLOAD.EMPTY))

# -----------------------------------------------------------------------------------------------

# print(robot.get_joint_angle_vel_limits(1))
# print(robot.set_joint_angle_vel_limits(joint_index=1, max_joint_spd=3.5))
# print(robot.get_joint_angle_vel_limits(1))


# print(robot.get_joint_acc_limits(1))
# print(robot.set_joint_acc_limits(max_joint_acc=5.0))
# print(robot.get_joint_acc_limits(1))


# print(robot.set_joint_angle_vel_acc_limits_to_default())


# print(robot.get_crash_protection_rating())
# print(robot.set_crash_protection_rating(joint_index=1, rating=1))
# print(robot.get_crash_protection_rating())


# print(end_effector.get_gripper_teaching_pendant_param())
# print(end_effector.set_gripper_teaching_pendant_param(teaching_range_per=100, max_range_config=0.07, teaching_friction=1))
# print(end_effector.get_gripper_teaching_pendant_param())


# print(robot.get_flange_vel_acc_limits())
# print(robot.set_flange_vel_acc_limits(0.5, 0.13, 0.8, 0.2))
# print(robot.set_flange_vel_acc_limits_to_default())
# print(robot.get_flange_vel_acc_limits())


# -------------------------- TCP ----------------------------

# robot.set_tcp_offset([0, 0, 0.15, 0, 0, 0])

# time.sleep(0.1)
# print(robot.get_tcp_pose())

# pose = robot.get_tcp2flange_pose([0.3, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])
# print(pose)
# robot.move_p(pose)
# time.sleep(2)
# robot.move_l([0.1, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])

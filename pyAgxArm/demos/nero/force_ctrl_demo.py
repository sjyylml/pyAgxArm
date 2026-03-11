"""Nero force control demo.

Demonstrates zero-gravity drag mode and compliant position control.

Usage:
    python -m pyAgxArm.demos.nero.force_ctrl_demo --channel can0
    python -m pyAgxArm.demos.nero.force_ctrl_demo --channel can0 --mode position
"""

import argparse
import logging
import time

import numpy as np

from pyAgxArm.protocols.can_protocol.drivers.nero.force_ctrl import (
    NeroForceController,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
)


def demo_zero_gravity(ctrl: NeroForceController, kd_scale: float = 1.0) -> None:
    """Zero-gravity free-drag mode.

    The arm floats freely with gravity compensation. You can move it by hand.
    """
    from pyAgxArm.protocols.can_protocol.drivers.nero.force_ctrl.controller import (
        ZERO_GRAV_KD, N_JOINTS,
    )
    from pyAgxArm.protocols.can_protocol.drivers.nero.force_ctrl.data_types import (
        JointCommands,
    )

    q = ctrl.get_joint_pos()
    if q is None:
        ctrl.zero_gravity_mode()
    else:
        kd = ZERO_GRAV_KD * kd_scale
        cmds = JointCommands(
            pos=q.copy(),
            vel=np.zeros(N_JOINTS),
            kp=np.zeros(N_JOINTS),
            kd=kd,
            torques=np.zeros(N_JOINTS),
        )
        ctrl._loop.update_commands(cmds)

    print("\n=== Zero-Gravity Mode ===")
    print(f"kd_scale={kd_scale}, kd={np.round(ZERO_GRAV_KD * kd_scale, 3).tolist()}")
    print("The arm is now in zero-gravity mode.")
    print("You can freely move the arm by hand.")
    print("Press Enter to switch to position hold...")

    try:
        while True:
            state = ctrl.get_joint_state()
            if state is not None:
                q_deg = np.degrees(state.pos)
                print(
                    f"\rJoints (deg): {np.array2string(q_deg, precision=1, suppress_small=True)}",
                    end="",
                    flush=True,
                )
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print()


def demo_position_hold(ctrl: NeroForceController) -> None:
    """Position hold with compliance.

    Locks the arm at its current position with configurable stiffness.
    """
    q = ctrl.get_joint_pos()
    if q is None:
        print("Error: no joint feedback")
        return

    print("\n=== Position Hold Mode ===")
    print(f"Holding position: {np.round(np.degrees(q), 1)} deg")
    print("The arm resists perturbation with compliance.")
    print("Press Ctrl+C to exit...")

    ctrl.command_joint_pos(q)

    try:
        while True:
            state = ctrl.get_joint_state()
            if state is not None:
                err = np.degrees(state.pos - q)
                print(
                    f"\rPosition error (deg): {np.array2string(err, precision=2, suppress_small=True)}",
                    end="",
                    flush=True,
                )
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print()


def demo_compliant_motion(ctrl: NeroForceController) -> None:
    """Compliant motion with low stiffness.

    Moves to a target with very low PD gains, allowing the arm to be
    easily pushed away from its trajectory.
    """
    q = ctrl.get_joint_pos()
    if q is None:
        print("Error: no joint feedback")
        return

    print("\n=== Compliant Motion Mode ===")
    print("Low-stiffness position control.")
    print("You can push the arm during motion.")
    print("Press Ctrl+C to exit...")

    # Low stiffness gains for compliant behavior
    soft_kp = np.array([5.0, 5.0, 5.0, 3.0, 2.0, 1.0, 1.0])
    soft_kd = np.array([2.0, 2.0, 2.0, 1.0, 0.5, 0.3, 0.3])

    ctrl.command_joint_state(
        pos=q,
        vel=np.zeros(7),
        kp=soft_kp,
        kd=soft_kd,
        torques=np.zeros(7),
    )

    try:
        while True:
            state = ctrl.get_joint_state()
            if state is not None:
                q_deg = np.degrees(state.pos)
                print(
                    f"\rJoints (deg): {np.array2string(q_deg, precision=1, suppress_small=True)}",
                    end="",
                    flush=True,
                )
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print()


def main():
    parser = argparse.ArgumentParser(description="Nero force control demo")
    parser.add_argument("--channel", default="can0", help="CAN channel")
    parser.add_argument("--interface", default="socketcan", help="CAN interface")
    parser.add_argument("--urdf", default="", help="Path to Nero URDF")
    parser.add_argument(
        "--mode",
        choices=["zero_gravity", "position", "compliant"],
        default="zero_gravity",
        help="Demo mode",
    )
    parser.add_argument(
        "--gravity_factor",
        type=float,
        default=1.0,
        help="Gravity compensation factor",
    )
    parser.add_argument(
        "--freq",
        type=int,
        default=200,
        help="Control loop frequency (Hz)",
    )
    parser.add_argument(
        "--kd_scale",
        type=float,
        default=1.0,
        help="Scale factor for zero-gravity damping kd (0.0 = no damping)",
    )
    args = parser.parse_args()

    ctrl = NeroForceController(
        channel=args.channel,
        urdf_path=args.urdf,
        zero_gravity=(args.mode == "zero_gravity"),
        gravity_factor=args.gravity_factor,
        control_freq=args.freq,
        interface=args.interface,
    )

    try:
        ctrl.start()

        if args.mode == "zero_gravity":
            demo_zero_gravity(ctrl, kd_scale=args.kd_scale)
        elif args.mode == "position":
            demo_position_hold(ctrl)
        elif args.mode == "compliant":
            demo_compliant_motion(ctrl)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        ctrl.stop()
        print("Done.")


if __name__ == "__main__":
    main()

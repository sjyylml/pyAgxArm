"""Verify whether the high-speed feedback current field is signed or unsigned.

Protocol doc (V1.2.1) marks byte[2:3] of 0x251-0x257 as uint16,
but the driver (decode_high_spd) parses it as int16.

This script captures raw CAN frames, shows both interpretations side-by-side,
collects statistics, and automatically judges the signedness.

Safety: uses MIT damping mode (kp=0, kd=0.3, t_ff=0) -- no active force.
The operator manually pushes the target joint in both directions to produce
positive/negative current.

Usage:
    python -m pyAgxArm.demos.nero.verify_current_sign --channel can0 --joint 7 --duration 15
    python -m pyAgxArm.demos.nero.verify_current_sign --channel can0
    python -m pyAgxArm.demos.nero.verify_current_sign --channel can0 --csv out.csv
"""

import argparse
import csv
import struct
import sys
import threading
import time
from collections import defaultdict
from typing import Dict, Optional

import can

from pyAgxArm.api import AgxArmFactory, create_agx_arm_config

# CAN IDs for high-speed motor feedback (joints 1-7)
HIGH_SPD_CAN_IDS = {0x251 + i: i + 1 for i in range(7)}  # {0x251:1, ..., 0x257:7}

# Column widths for display
COL_W = 14

# int16 extreme thresholds (±32.767 / ±32.768 A)
_INT16_SAT_THRESHOLD = 32000  # raw int16 abs value considered "saturated"


class JointStats:
    """Per-joint statistics collected during the test."""

    __slots__ = (
        "sample_count",
        "uint16_min", "uint16_max",
        "int16_min", "int16_max",
        "driver_min", "driver_max",
        "seen_above_8000",
        "seen_positive", "seen_negative",
        "saturated_count",
    )

    def __init__(self):
        self.sample_count = 0
        self.uint16_min = 0xFFFF
        self.uint16_max = 0
        self.int16_min = 32767
        self.int16_max = -32768
        self.driver_min = float("inf")
        self.driver_max = float("-inf")
        self.seen_above_8000 = False
        self.seen_positive = False
        self.seen_negative = False
        self.saturated_count = 0

    def update(self, uint16_val: int, int16_val: int, driver_val: float):
        self.sample_count += 1
        self.uint16_min = min(self.uint16_min, uint16_val)
        self.uint16_max = max(self.uint16_max, uint16_val)
        self.int16_min = min(self.int16_min, int16_val)
        self.int16_max = max(self.int16_max, int16_val)
        self.driver_min = min(self.driver_min, driver_val)
        self.driver_max = max(self.driver_max, driver_val)
        if uint16_val > 0x8000:
            self.seen_above_8000 = True
        if int16_val > 0:
            self.seen_positive = True
        if int16_val < 0:
            self.seen_negative = True
        if abs(int16_val) >= _INT16_SAT_THRESHOLD:
            self.saturated_count += 1


class CurrentRecord:
    """Latest raw current data for one joint."""

    __slots__ = ("joint", "raw_hex", "uint16_val", "int16_val", "driver_val", "timestamp")

    def __init__(self, joint: int):
        self.joint = joint
        self.raw_hex = "----"
        self.uint16_val = 0
        self.int16_val = 0
        self.driver_val = 0.0
        self.timestamp = 0.0


def parse_args():
    p = argparse.ArgumentParser(
        description="Verify high-speed feedback current field signedness"
    )
    p.add_argument("--channel", default="can0", help="CAN channel (default: can0)")
    p.add_argument("--interface", default="socketcan", help="CAN interface type")
    p.add_argument(
        "--joint",
        type=int,
        default=0,
        help="Only show this joint (1-7), 0 = all (default: 0)",
    )
    p.add_argument("--csv", default="", help="Path to CSV output file (optional)")
    p.add_argument(
        "--duration",
        type=float,
        default=15,
        help="Auto-exit after N seconds (default: 15, 0 = manual Ctrl+C)",
    )
    return p.parse_args()


def _judge_joint(s: JointStats) -> str:
    """Return a verdict string for one joint based on collected stats.

    Verdicts
    --------
    SIGNED           uint16 crossed 0x8000 — field is signed int16.
    UNSIGNED         uint16 stayed in [0, 0x7FFF] even though values were
                     non-trivial — field is likely unsigned uint16.
    INCONCLUSIVE     Not enough data or the joint was not moved.
    """
    if s.sample_count == 0:
        return "NO_DATA"

    if s.seen_above_8000:
        return "SIGNED"

    # uint16 never exceeded 0x8000.
    # If we saw meaningful variation (> 100 raw units) the field is likely unsigned.
    uint16_range = s.uint16_max - s.uint16_min
    if uint16_range > 100:
        return "UNSIGNED"

    return "INCONCLUSIVE"


def _print_judgment(joints_to_show, stats: Dict[int, "JointStats"]):
    """Print per-joint statistics and the automatic verdict."""
    W = 72
    print()
    print("=" * W)
    print("  STATISTICS & AUTO-JUDGMENT")
    print("=" * W)

    for j in joints_to_show:
        s = stats[j]
        if s.sample_count == 0:
            print(f"\n  Joint {j}: no data received")
            continue

        verdict = _judge_joint(s)
        sat_pct = (
            f"{s.saturated_count / s.sample_count * 100:.1f}%"
            if s.sample_count
            else "N/A"
        )

        print(f"\n  Joint {j}  ({s.sample_count} samples)")
        print(f"    uint16  range : {s.uint16_min:>6} ~ {s.uint16_max:<6}"
              f"  (0x{s.uint16_min:04X} ~ 0x{s.uint16_max:04X})")
        print(f"    int16   range : {s.int16_min:>6} ~ {s.int16_max:<6}")
        print(f"    driver  range : {s.driver_min:>+10.4f} ~ {s.driver_max:<+10.4f} A")
        print(f"    uint16 > 0x8000 ever seen : {'YES' if s.seen_above_8000 else 'NO'}")
        print(f"    positive int16 seen       : {'YES' if s.seen_positive else 'NO'}")
        print(f"    negative int16 seen       : {'YES' if s.seen_negative else 'NO'}")
        print(f"    |int16| >= {_INT16_SAT_THRESHOLD} (saturated) : "
              f"{s.saturated_count}/{s.sample_count} ({sat_pct})")
        print()

        if verdict == "SIGNED":
            print(f"    >> VERDICT: SIGNED int16")
            print(f"       Driver code (ConvertToNegative_16bit signed=True) is CORRECT.")
            print(f"       Protocol doc V1.2.1 labelling it as uint16 is INACCURATE.")
            if s.saturated_count > 0:
                print()
                print(f"    >> WARNING: {sat_pct} of samples hit int16 extreme "
                      f"(|value| >= {_INT16_SAT_THRESHOLD}).")
                print(f"       driver range = [{s.driver_min:+.4f}, {s.driver_max:+.4f}] A")
                print(f"       ±32.767 A is the int16 limit — this may indicate")
                print(f"       sensor saturation, or a different scale for this joint.")
        elif verdict == "UNSIGNED":
            print(f"    >> VERDICT: likely UNSIGNED uint16")
            print(f"       uint16 values stayed within [0, 0x7FFF].")
            print(f"       Consider changing decode to signed=False for this joint.")
        else:
            print(f"    >> VERDICT: INCONCLUSIVE")
            print(f"       Not enough variation observed.")
            print(f"       Try pushing the joint harder in both directions.")

    print()
    print("=" * W)


def main():
    args = parse_args()

    # ------------------------------------------------------------------
    # 1. Create robot driver
    # ------------------------------------------------------------------
    config = create_agx_arm_config(
        robot="nero",
        comm="can",
        channel=args.channel,
        interface=args.interface,
    )
    robot = AgxArmFactory.create_arm(config)

    # ------------------------------------------------------------------
    # 2. Shared state for raw CAN capture
    # ------------------------------------------------------------------
    records: Dict[int, CurrentRecord] = {j: CurrentRecord(j) for j in range(1, 8)}
    lock = threading.Lock()

    csv_file = None
    csv_writer = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(
            ["timestamp", "joint", "raw_hex", "uint16", "int16", "driver_current_A"]
        )

    # ------------------------------------------------------------------
    # 3. Register raw CAN callback BEFORE connect
    # ------------------------------------------------------------------
    def on_can_frame(msg: can.Message):
        cid = msg.arbitration_id
        if cid not in HIGH_SPD_CAN_IDS:
            return
        joint = HIGH_SPD_CAN_IDS[cid]

        raw_bytes = bytes(msg.data[2:4])
        uint16_val = struct.unpack(">H", raw_bytes)[0]  # big-endian unsigned
        int16_val = struct.unpack(">h", raw_bytes)[0]  # big-endian signed

        with lock:
            r = records[joint]
            r.raw_hex = raw_bytes.hex().upper()
            r.uint16_val = uint16_val
            r.int16_val = int16_val
            r.timestamp = msg.timestamp

    robot.get_context().register_parser_packet_fun(on_can_frame)

    # ------------------------------------------------------------------
    # 4. Connect and initialise
    # ------------------------------------------------------------------
    print("[1/5] Connecting to CAN bus ...")
    robot.connect()
    time.sleep(0.5)

    print("[2/5] Setting normal mode ...")
    robot.set_normal_mode()
    time.sleep(0.5)

    print("[3/5] Enabling all joints ...")
    robot.enable()
    time.sleep(1.0)

    print("[4/5] Entering MIT damping mode (kp=0, kd=0.3, t_ff=0) ...")
    robot.set_motion_mode("mit")
    time.sleep(0.3)

    # Send light damping to all joints so the arm doesn't free-fall
    for j in range(1, 8):
        robot.move_mit(joint_index=j, kp=0, kd=0.3, t_ff=0)
    time.sleep(0.5)

    print("[5/5] Ready.")
    print()
    print("=" * 70)
    print("  CURRENT FIELD SIGN VERIFICATION")
    print("=" * 70)
    print()
    print("Instructions:")
    print("  - The arm is in light-damping mode (no active motion).")
    print("  - Gently push joint 7 (wrist rotation) in BOTH directions.")
    print("  - Observe whether raw uint16 values cross 0x8000.")
    print()
    print("Interpretation:")
    print("  - If uint16 > 0x8000 appears  => field is SIGNED (code is correct)")
    print("  - If uint16 always < 0x8000   => field may be UNSIGNED (doc is correct)")
    print()
    print("Press Ctrl+C to stop.\n")

    # ------------------------------------------------------------------
    # 5. Display loop
    # ------------------------------------------------------------------
    joints_to_show = list(range(1, 8))
    if args.joint and 1 <= args.joint <= 7:
        joints_to_show = [args.joint]

    # Per-joint statistics
    stats: Dict[int, JointStats] = {j: JointStats() for j in range(1, 8)}

    header = (
        f"{'Joint':>{COL_W}}"
        f"{'RawHex':>{COL_W}}"
        f"{'uint16':>{COL_W}}"
        f"{'int16':>{COL_W}}"
        f"{'driver(A)':>{COL_W}}"
        f"{'> 0x8000?':>{COL_W}}"
    )

    dur_str = f"{args.duration:.0f}s" if args.duration > 0 else "manual"
    print(f"Collecting data ({dur_str}) — push the joint in BOTH directions ...\n")

    start_time = time.time()
    line_count = 0

    try:
        while True:
            # Keep sending damping commands to stay in MIT mode
            for j in range(1, 8):
                robot.move_mit(joint_index=j, kp=0, kd=0.3, t_ff=0)

            # Read driver-layer values
            with lock:
                for j in joints_to_show:
                    ms = robot.get_motor_states(j)
                    if ms is not None:
                        records[j].driver_val = ms.msg.current

            # Print header periodically
            if line_count % 20 == 0:
                print(header)
                print("-" * (COL_W * 6))

            with lock:
                for j in joints_to_show:
                    r = records[j]
                    above = r.uint16_val > 0x8000
                    flag = "YES **" if above else ""

                    # Update statistics
                    stats[j].update(r.uint16_val, r.int16_val, r.driver_val)

                    line = (
                        f"{j:>{COL_W}}"
                        f"{r.raw_hex:>{COL_W}}"
                        f"{r.uint16_val:>{COL_W}}"
                        f"{r.int16_val:>{COL_W}}"
                        f"{r.driver_val:>{COL_W}.4f}"
                        f"{flag:>{COL_W}}"
                    )
                    print(line)

                    if csv_writer:
                        csv_writer.writerow([
                            f"{r.timestamp:.6f}",
                            j,
                            r.raw_hex,
                            r.uint16_val,
                            r.int16_val,
                            f"{r.driver_val:.6f}",
                        ])

            line_count += 1
            time.sleep(0.1)

            if args.duration > 0 and (time.time() - start_time) > args.duration:
                print(f"\nDuration {args.duration}s reached, stopping.")
                break

    except KeyboardInterrupt:
        print("\n\nCtrl+C received, stopping ...")

    # ------------------------------------------------------------------
    # 6. Auto-judgment
    # ------------------------------------------------------------------
    _print_judgment(joints_to_show, stats)

    # ------------------------------------------------------------------
    # 7. Safe shutdown
    # ------------------------------------------------------------------
    print("Emergency stopping ...")
    try:
        robot.electronic_emergency_stop()
        time.sleep(0.3)
        robot.disable()
    except Exception as e:
        print(f"  shutdown warning: {e}")

    if csv_file:
        csv_file.close()
        print(f"CSV saved to: {args.csv}")

    print("Done.")


if __name__ == "__main__":
    main()

import copy
import time
import logging
import threading
import numpy as np

from .data_types import JointCommands, JointState
from .gravity_comp import NeroGravityCompensator

logger = logging.getLogger(__name__)

# Nero joint limits (rad) from api/constants.py
NERO_JOINT_LIMITS = np.array([
    [-2.705261, 2.705261],   # joint1
    [-1.745330, 1.745330],   # joint2
    [-2.757621, 2.757621],   # joint3
    [-1.012291, 2.146755],   # joint4
    [-2.757621, 2.757621],   # joint5
    [-0.733039, 0.959932],   # joint6
    [-1.570797, 1.570797],   # joint7
])

# Safety buffer around joint limits (rad)
JOINT_LIMIT_BUFFER = 0.1

# Max position jump between consecutive frames (rad)
MAX_POS_JUMP = 0.5

# Report interval for loop statistics (seconds)
REPORT_INTERVAL = 30.0


class NeroMITLoop:
    """200 Hz real-time MIT control loop for Nero 7-DOF arm.

    This loop runs in a background thread, continuously:
    1. Reads the latest joint angles from the pyAgxArm driver (parser cache)
    2. Computes gravity compensation torques via MuJoCo
    3. Combines user commands with gravity compensation
    4. Sends MIT control CAN frames for all 7 joints

    Parameters
    ----------
    robot : Driver
        The Nero pyAgxArm driver instance (already connected).
    gravity_comp : NeroGravityCompensator
        The gravity compensator instance.
    control_freq : int
        Control loop frequency in Hz. Default 200.
    gravity_factor : float
        Multiplier for gravity compensation torques. Default 1.0.
    torque_limit : float
        Maximum absolute torque per joint (N·m). Default 8.0.
    miss_limit : int
        Maximum consecutive feedback misses before emergency stop. Default 20.
    """

    N_JOINTS = 7

    def __init__(
        self,
        robot,
        gravity_comp: NeroGravityCompensator,
        control_freq: int = 200,
        gravity_factor: float = 1.0,
        torque_limit: float = 8.0,
        miss_limit: int = 20,
    ):
        self._robot = robot
        self._grav = gravity_comp
        self._freq = control_freq
        self._period = 1.0 / control_freq
        self._gravity_factor = gravity_factor
        self._torque_limit = torque_limit
        self._miss_limit = miss_limit

        # Thread-safe command buffer
        self._command_lock = threading.Lock()
        self._commands = JointCommands.zeros(self.N_JOINTS)

        # Thread-safe state buffer
        self._state_lock = threading.Lock()
        self._joint_state: JointState = None

        self._running = False
        self._thread: threading.Thread = None

        # MIT mode initialized flag
        self._mit_mode_set = False

    @property
    def running(self) -> bool:
        return self._running

    def start(self) -> None:
        """Start the control loop in a background thread."""
        if self._running:
            logger.warning("MIT loop already running")
            return

        self._running = True
        self._mit_mode_set = False
        self._thread = threading.Thread(
            target=self._control_loop, daemon=True, name="nero_mit_loop"
        )
        self._thread.start()
        logger.info(f"MIT control loop started at {self._freq} Hz")

    def stop(self) -> None:
        """Stop the control loop."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._mit_mode_set = False
        logger.info("MIT control loop stopped")

    def update_commands(self, commands: JointCommands) -> None:
        """Update the control commands (thread-safe)."""
        with self._command_lock:
            self._commands = copy.deepcopy(commands)

    def get_joint_state(self) -> JointState:
        """Get the latest joint state (thread-safe)."""
        with self._state_lock:
            if self._joint_state is None:
                return None
            return copy.deepcopy(self._joint_state)

    def _read_joint_pos(self) -> np.ndarray:
        """Read current joint positions from the driver's parser cache."""
        ja = self._robot.get_joint_angles()
        if ja is None:
            return None
        return np.array(ja.msg, dtype=np.float64)

    def _read_joint_vel(self) -> np.ndarray:
        """Read current joint velocities from motor state feedback."""
        vel = np.zeros(self.N_JOINTS)
        for i in range(1, self.N_JOINTS + 1):
            ms = self._robot.get_motor_states(i)
            if ms is not None:
                vel[i - 1] = ms.msg.motor_speed
        return vel

    def _read_joint_torque(self) -> np.ndarray:
        """Read current joint torques from motor state feedback."""
        torque = np.zeros(self.N_JOINTS)
        for i in range(1, self.N_JOINTS + 1):
            ms = self._robot.get_motor_states(i)
            if ms is not None:
                torque[i - 1] = ms.msg.torque
        return torque

    def _check_joint_limits(self, q: np.ndarray) -> bool:
        """Check if joint positions are within limits (with buffer).

        Returns True if safe, False if violated.
        """
        lower = NERO_JOINT_LIMITS[:, 0] - JOINT_LIMIT_BUFFER
        upper = NERO_JOINT_LIMITS[:, 1] + JOINT_LIMIT_BUFFER
        if np.any(q < lower) or np.any(q > upper):
            violations = []
            for i in range(self.N_JOINTS):
                if q[i] < lower[i] or q[i] > upper[i]:
                    violations.append(
                        f"joint{i+1}={q[i]:.4f} "
                        f"limit=[{NERO_JOINT_LIMITS[i,0]:.4f}, "
                        f"{NERO_JOINT_LIMITS[i,1]:.4f}]"
                    )
            logger.error(f"Joint limit violation: {', '.join(violations)}")
            return False
        return True

    def _send_mit_commands(
        self, pos, vel, kp, kd, torques
    ) -> None:
        """Send MIT commands for all 7 joints."""
        # Set MIT mode once at the start
        if not self._mit_mode_set:
            self._robot.set_motion_mode('mit')
            self._mit_mode_set = True

        commands = []
        for i in range(self.N_JOINTS):
            commands.append({
                "joint_index": i + 1,
                "p_des": float(pos[i]),
                "v_des": float(vel[i]),
                "kp": float(kp[i]),
                "kd": float(kd[i]),
                "t_ff": float(torques[i]),
            })
        self._robot.move_mit_batch(commands)

    def _control_loop(self) -> None:
        """Main control loop (runs in background thread)."""
        miss_count = 0
        prev_q = None
        step_count = 0
        exceed_count = 0
        report_start = time.time()

        while self._running:
            loop_start = time.perf_counter()

            try:
                # 1. Read latest joint state
                q = self._read_joint_pos()

                if q is None:
                    miss_count += 1
                    if miss_count > self._miss_limit:
                        logger.error(
                            f"Lost joint feedback for {miss_count} "
                            f"consecutive cycles, emergency stop"
                        )
                        self._running = False
                        self._robot.electronic_emergency_stop()
                        break
                    # Use previous state and skip control
                    self._sleep_until(loop_start + self._period)
                    continue

                miss_count = 0

                # 2. Safety: position jump detection
                if prev_q is not None:
                    dq = np.abs(q - prev_q)
                    if np.any(dq > MAX_POS_JUMP):
                        logger.warning(
                            f"Position jump detected: max_dq="
                            f"{np.max(dq):.4f} rad"
                        )
                prev_q = q.copy()

                # 3. Safety: joint limit check
                if not self._check_joint_limits(q):
                    logger.error("Joint limit exceeded, emergency stop")
                    self._running = False
                    self._robot.electronic_emergency_stop()
                    break

                # 4. Compute gravity compensation
                grav_torque = self._grav.compute(q) * self._gravity_factor

                # 5. Read user commands
                with self._command_lock:
                    cmds = copy.deepcopy(self._commands)

                # 6. Combine: total_torque = user_torque + gravity
                total_torque = cmds.torques + grav_torque

                # 7. Clamp torques
                total_torque = np.clip(
                    total_torque,
                    -self._torque_limit,
                    self._torque_limit,
                )

                # 8. Clamp position commands to joint limits
                cmd_pos = np.clip(
                    cmds.pos,
                    NERO_JOINT_LIMITS[:, 0],
                    NERO_JOINT_LIMITS[:, 1],
                )

                # 9. Send MIT commands for all joints
                self._send_mit_commands(
                    pos=cmd_pos,
                    vel=cmds.vel,
                    kp=cmds.kp,
                    kd=cmds.kd,
                    torques=total_torque,
                )

                # 10. Update joint state for external readers
                vel = self._read_joint_vel()
                torque = self._read_joint_torque()
                with self._state_lock:
                    self._joint_state = JointState(
                        pos=q, vel=vel, torque=torque,
                        timestamp=time.time(),
                    )

            except RuntimeError as e:
                logger.error(f"Control loop error: {e}")
                self._running = False
                try:
                    self._robot.electronic_emergency_stop()
                except Exception:
                    pass
                break

            # 11. Rate limiting
            elapsed = time.perf_counter() - loop_start
            step_count += 1
            if elapsed > self._period * 1.5:
                exceed_count += 1

            # Periodic reporting
            now = time.time()
            if now - report_start >= REPORT_INTERVAL:
                freq = step_count / (now - report_start) if step_count > 0 else 0
                logger.info(
                    f"MIT loop: {freq:.1f} Hz, "
                    f"exceeded={exceed_count}/{step_count}"
                )
                step_count = 0
                exceed_count = 0
                report_start = now

            self._sleep_until(loop_start + self._period)

    def _sleep_until(self, target_time: float) -> None:
        """Sleep until target_time using a combination of busy-wait and sleep."""
        remaining = target_time - time.perf_counter()
        if remaining > 0.002:
            time.sleep(remaining - 0.001)
        # Busy-wait for the last ~1ms for precision
        while time.perf_counter() < target_time:
            pass

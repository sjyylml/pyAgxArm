import time
import logging
import numpy as np

from pyAgxArm import create_agx_arm_config, AgxArmFactory
from .gravity_comp import NeroGravityCompensator
from .mit_loop import NeroMITLoop, NERO_JOINT_LIMITS
from .data_types import JointCommands, JointState

logger = logging.getLogger(__name__)

# Default PD gains (conservative starting values, tune on real hardware)
DEFAULT_KP = np.array([30.0, 30.0, 30.0, 15.0, 10.0, 5.0, 5.0])
DEFAULT_KD = np.array([2.0, 2.0, 2.0, 1.0, 0.5, 0.3, 0.3])

# Damping for zero-gravity mode (prevents free oscillation)
ZERO_GRAV_KD = np.array([1.0, 1.0, 1.0, 0.5, 0.3, 0.2, 0.2])

N_JOINTS = 7


class NeroForceController:
    """High-level force control interface for the Nero 7-DOF arm.

    Wraps the pyAgxArm Nero driver with a real-time MIT control loop
    and MuJoCo-based gravity compensation.

    Parameters
    ----------
    channel : str
        CAN channel name (e.g. "can0").
    urdf_path : str, optional
        Path to Nero URDF for gravity compensation.
        Defaults to the bundled model.
    zero_gravity : bool, optional
        If True, start in zero-gravity (free-drag) mode.
        If False, start in position-hold mode at current pose.
        Default True.
    gravity_factor : float, optional
        Gravity compensation multiplier. Default 1.0.
    control_freq : int, optional
        MIT control loop frequency in Hz. Default 200.
    torque_limit : float, optional
        Maximum absolute torque per joint (N·m). Default 8.0.
    interface : str, optional
        CAN interface type. Default "socketcan".

    Examples
    --------
    Zero-gravity drag mode::

        ctrl = NeroForceController(channel="can0", zero_gravity=True)
        ctrl.start()
        # The arm can now be moved freely by hand.
        # ...
        ctrl.stop()

    Position hold with compliance::

        ctrl = NeroForceController(channel="can0", zero_gravity=False)
        ctrl.start()
        # Arm holds its initial position.
        ctrl.command_joint_pos(target_q, kp=[10]*7, kd=[1]*7)
        # ...
        ctrl.stop()
    """

    def __init__(
        self,
        channel: str = "can0",
        urdf_path: str = "",
        zero_gravity: bool = True,
        gravity_factor: float = 1.0,
        control_freq: int = 200,
        torque_limit: float = 8.0,
        interface: str = "socketcan",
    ):
        self._channel = channel
        self._zero_gravity_init = zero_gravity
        self._gravity_factor = gravity_factor
        self._control_freq = control_freq
        self._torque_limit = torque_limit

        # Create pyAgxArm Nero driver
        cfg = create_agx_arm_config(
            robot="nero",
            comm="can",
            channel=channel,
            interface=interface,
        )
        self._robot = AgxArmFactory.create_arm(cfg)

        # Create gravity compensator
        self._grav = NeroGravityCompensator(
            urdf_path=urdf_path,
            gravity=np.array([0.0, 0.0, -9.81]),
            max_torque_sanity=20.0,
        )

        # Create MIT control loop
        self._loop = NeroMITLoop(
            robot=self._robot,
            gravity_comp=self._grav,
            control_freq=control_freq,
            gravity_factor=gravity_factor,
            torque_limit=torque_limit,
        )

        # Store default gains
        self._kp = DEFAULT_KP.copy()
        self._kd = DEFAULT_KD.copy()

        self._started = False

    @property
    def robot(self):
        """Access the underlying pyAgxArm Nero driver."""
        return self._robot

    def start(self) -> None:
        """Connect, enable, and start the control loop.

        This method blocks until the arm is enabled and the first joint
        feedback is received.
        """
        if self._started:
            logger.warning("Controller already started")
            return

        # Connect to the arm
        self._robot.connect()
        logger.info(f"Connected to Nero on {self._channel}")

        # Set normal mode and enable
        self._robot.set_normal_mode()
        timeout = time.time() + 5.0
        while not self._robot.enable():
            if time.time() > timeout:
                raise RuntimeError("Failed to enable Nero arm within 5 seconds")
            time.sleep(0.01)
        logger.info("Nero arm enabled")

        # Wait for first feedback
        q = None
        timeout = time.time() + 3.0
        while q is None:
            ja = self._robot.get_joint_angles()
            if ja is not None:
                q = np.array(ja.msg, dtype=np.float64)
            if time.time() > timeout:
                raise RuntimeError("No joint feedback received within 3 seconds")
            time.sleep(0.02)
        logger.info(f"Initial joint angles: {np.round(q, 4).tolist()}")

        # Set initial mode
        if self._zero_gravity_init:
            cmds = JointCommands(
                pos=q.copy(),
                vel=np.zeros(N_JOINTS),
                kp=np.zeros(N_JOINTS),
                kd=ZERO_GRAV_KD.copy(),
                torques=np.zeros(N_JOINTS),
            )
        else:
            cmds = JointCommands(
                pos=q.copy(),
                vel=np.zeros(N_JOINTS),
                kp=self._kp.copy(),
                kd=self._kd.copy(),
                torques=np.zeros(N_JOINTS),
            )

        self._loop.update_commands(cmds)
        self._loop.start()
        self._started = True
        mode_name = "zero-gravity" if self._zero_gravity_init else "position-hold"
        logger.info(f"Control loop started in {mode_name} mode")

    def stop(self) -> None:
        """Stop the control loop and disable the arm."""
        if not self._started:
            return

        self._loop.stop()

        try:
            self._robot.electronic_emergency_stop()
        except Exception:
            pass
        time.sleep(0.1)

        try:
            timeout = time.time() + 2.0
            while not self._robot.disable():
                if time.time() > timeout:
                    break
                time.sleep(0.01)
        except Exception:
            pass

        self._started = False
        logger.info("Nero force controller stopped")

    # ========================= Control Modes =========================

    def zero_gravity_mode(self) -> None:
        """Switch to zero-gravity (free drag) mode.

        In this mode, only gravity compensation torques are applied.
        A small damping (kd) is used to prevent free oscillation.
        The arm can be freely moved by hand.
        """
        q = self.get_joint_pos()
        if q is None:
            raise RuntimeError("No joint feedback available")

        cmds = JointCommands(
            pos=q.copy(),
            vel=np.zeros(N_JOINTS),
            kp=np.zeros(N_JOINTS),
            kd=ZERO_GRAV_KD.copy(),
            torques=np.zeros(N_JOINTS),
        )
        self._loop.update_commands(cmds)
        logger.info("Switched to zero-gravity mode")

    def zero_torque_mode(self) -> None:
        """Switch to zero-torque (fully passive) mode.

        All PD gains and torques are zero. The arm is completely passive
        with no gravity compensation. Use with caution - the arm will fall.
        """
        cmds = JointCommands.zeros(N_JOINTS)
        self._loop.update_commands(cmds)
        logger.info("Switched to zero-torque mode (no gravity comp)")

    def command_joint_pos(
        self,
        pos: np.ndarray,
        kp: np.ndarray = None,
        kd: np.ndarray = None,
    ) -> None:
        """Position PD control with gravity compensation.

        Parameters
        ----------
        pos : array-like, shape (7,)
            Target joint angles in radians.
        kp : array-like, shape (7,), optional
            Proportional gains. Default uses DEFAULT_KP.
        kd : array-like, shape (7,), optional
            Derivative gains. Default uses DEFAULT_KD.
        """
        pos = np.asarray(pos, dtype=np.float64).ravel()
        if len(pos) != N_JOINTS:
            raise ValueError(f"Expected {N_JOINTS} joint angles, got {len(pos)}")

        # Clamp to joint limits
        pos = np.clip(pos, NERO_JOINT_LIMITS[:, 0], NERO_JOINT_LIMITS[:, 1])

        if kp is None:
            kp = self._kp.copy()
        else:
            kp = np.asarray(kp, dtype=np.float64).ravel()

        if kd is None:
            kd = self._kd.copy()
        else:
            kd = np.asarray(kd, dtype=np.float64).ravel()

        cmds = JointCommands(
            pos=pos,
            vel=np.zeros(N_JOINTS),
            kp=kp,
            kd=kd,
            torques=np.zeros(N_JOINTS),
        )
        self._loop.update_commands(cmds)

    def command_joint_state(
        self,
        pos: np.ndarray,
        vel: np.ndarray = None,
        kp: np.ndarray = None,
        kd: np.ndarray = None,
        torques: np.ndarray = None,
    ) -> None:
        """Full MIT control with gravity compensation auto-added.

        Parameters
        ----------
        pos : array-like, shape (7,)
            Target joint positions in radians.
        vel : array-like, shape (7,), optional
            Target joint velocities in rad/s. Default zeros.
        kp : array-like, shape (7,), optional
            Proportional gains. Default uses DEFAULT_KP.
        kd : array-like, shape (7,), optional
            Derivative gains. Default uses DEFAULT_KD.
        torques : array-like, shape (7,), optional
            User feed-forward torques in N·m (gravity compensation
            is added on top automatically). Default zeros.
        """
        pos = np.asarray(pos, dtype=np.float64).ravel()
        if len(pos) != N_JOINTS:
            raise ValueError(f"Expected {N_JOINTS} joint angles, got {len(pos)}")
        pos = np.clip(pos, NERO_JOINT_LIMITS[:, 0], NERO_JOINT_LIMITS[:, 1])

        if vel is None:
            vel = np.zeros(N_JOINTS)
        else:
            vel = np.asarray(vel, dtype=np.float64).ravel()

        if kp is None:
            kp = self._kp.copy()
        else:
            kp = np.asarray(kp, dtype=np.float64).ravel()

        if kd is None:
            kd = self._kd.copy()
        else:
            kd = np.asarray(kd, dtype=np.float64).ravel()

        if torques is None:
            torques = np.zeros(N_JOINTS)
        else:
            torques = np.asarray(torques, dtype=np.float64).ravel()

        cmds = JointCommands(
            pos=pos,
            vel=vel,
            kp=kp,
            kd=kd,
            torques=torques,
        )
        self._loop.update_commands(cmds)

    def update_gains(
        self,
        kp: np.ndarray = None,
        kd: np.ndarray = None,
    ) -> None:
        """Update the default PD gains.

        Parameters
        ----------
        kp : array-like, shape (7,), optional
        kd : array-like, shape (7,), optional
        """
        if kp is not None:
            self._kp = np.asarray(kp, dtype=np.float64).ravel()
        if kd is not None:
            self._kd = np.asarray(kd, dtype=np.float64).ravel()

    # ========================= State Reading =========================

    def get_joint_state(self) -> JointState:
        """Get the latest joint state.

        Returns
        -------
        JointState or None
            Contains pos (7,), vel (7,), torque (7,), timestamp.
        """
        return self._loop.get_joint_state()

    def get_joint_pos(self) -> np.ndarray:
        """Get current joint positions.

        Returns
        -------
        np.ndarray, shape (7,) or None
        """
        ja = self._robot.get_joint_angles()
        if ja is not None:
            return np.array(ja.msg, dtype=np.float64)
        return None

    # ========================= Safety =========================

    def emergency_stop(self) -> None:
        """Trigger a damped emergency stop and halt the control loop."""
        self._loop.stop()
        try:
            self._robot.electronic_emergency_stop()
        except Exception:
            pass
        logger.warning("Emergency stop triggered")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        return False

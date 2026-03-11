import os
import re
import logging
import numpy as np

try:
    import mujoco
except ImportError as exc:
    raise ImportError(
        "mujoco is required for gravity compensation. "
        "Install with: pip install mujoco"
    ) from exc

logger = logging.getLogger(__name__)

# Default URDF bundled with pyAgxArm
# Navigate from force_ctrl/ up to pyAgxArm/ package root:
#   force_ctrl -> nero -> drivers -> can_protocol -> protocols -> pyAgxArm
_PACKAGE_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..")
)
_DEFAULT_URDF = os.path.join(_PACKAGE_ROOT, "robot_models", "nero", "nero_description.urdf")


class NeroGravityCompensator:
    """MuJoCo-based gravity compensation for the Nero 7-DOF arm.

    Uses ``mujoco.mj_inverse`` to compute the joint torques needed to
    counteract gravity at a given joint configuration.

    Parameters
    ----------
    urdf_path : str, optional
        Path to the Nero URDF file.  Defaults to the bundled model under
        ``pyAgxArm/robot_models/nero/nero_description.urdf``.
    gravity : array-like, optional
        Gravity vector in the world frame.  Default ``[0, 0, -9.81]``.
    max_torque_sanity : float, optional
        If any computed torque exceeds this value, a RuntimeError is raised
        as a configuration-error safeguard.  Default 20.0 N·m.
    """

    def __init__(
        self,
        urdf_path: str = "",
        gravity: np.ndarray = None,
        max_torque_sanity: float = 20.0,
    ):
        if not urdf_path:
            urdf_path = os.path.normpath(_DEFAULT_URDF)

        if not os.path.isfile(urdf_path):
            raise FileNotFoundError(f"URDF not found: {urdf_path}")

        # For gravity compensation we only need inertial properties and
        # joint structure.  Strip <visual> and <collision> elements so
        # MuJoCo doesn't try to resolve ROS package:// mesh URIs.
        with open(urdf_path, "r") as f:
            urdf_xml = f.read()
        urdf_xml = re.sub(
            r"<visual>.*?</visual>", "", urdf_xml, flags=re.DOTALL
        )
        urdf_xml = re.sub(
            r"<collision>.*?</collision>", "", urdf_xml, flags=re.DOTALL
        )
        self._model = mujoco.MjModel.from_xml_string(urdf_xml)
        self._data = mujoco.MjData(self._model)
        self._max_torque = max_torque_sanity

        # Set gravity
        if gravity is not None:
            self._model.opt.gravity[:] = np.asarray(gravity, dtype=np.float64)
        else:
            self._model.opt.gravity[:] = [0.0, 0.0, -9.81]

        # Disable joint limits in MuJoCo (we enforce limits separately)
        self._model.jnt_limited[:] = 0

        self._nq = self._model.nq
        logger.info(
            f"NeroGravityCompensator loaded: {urdf_path}, "
            f"nq={self._nq}, gravity={self._model.opt.gravity.tolist()}"
        )

    @property
    def nq(self) -> int:
        return self._nq

    def compute(self, q: np.ndarray) -> np.ndarray:
        """Compute gravity compensation torques.

        Parameters
        ----------
        q : np.ndarray
            Joint angles in radians, shape ``(nq,)``.

        Returns
        -------
        np.ndarray
            Gravity compensation torques, shape ``(nq,)``.
            Positive torque counteracts downward gravity.
        """
        q = np.asarray(q, dtype=np.float64).ravel()
        if len(q) != self._nq:
            raise ValueError(
                f"Expected {self._nq} joint angles, got {len(q)}"
            )

        self._data.qpos[:self._nq] = q
        self._data.qvel[:self._nq] = 0.0
        self._data.qacc[:self._nq] = 0.0

        mujoco.mj_inverse(self._model, self._data)
        tau = self._data.qfrc_inverse[:self._nq].copy()

        if np.max(np.abs(tau)) > self._max_torque:
            raise RuntimeError(
                f"Gravity compensation torque too large: "
                f"max={np.max(np.abs(tau)):.2f} > {self._max_torque:.2f} N·m. "
                f"Check URDF or joint configuration."
            )

        return tau

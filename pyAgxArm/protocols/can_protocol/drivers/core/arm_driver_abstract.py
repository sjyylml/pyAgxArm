import threading
from typing import Optional, TYPE_CHECKING, overload, List

from typing_extensions import Literal, Final
from .arm_driver_interface import ArmDriverInterface
from .driver_context import DriverContext
from ...msgs.core import AttributeBase, MessageAbstract
from .protocol_parser_interface import ProtocolParserInterface
from ..core.arm_driver_context import ArmDriverContext
from .....utiles.vaildator import Validator
from .....utiles.tf import (
    pose6_to_T,
    matmul4,
    inv_T,
    T_to_pose6
)

if TYPE_CHECKING:
    from ..effector.agx_gripper import AgxGripperDriverDefault
    from ..effector.revo2 import Revo2DriverDefault


class ArmDriverAbstract(ArmDriverInterface):
    _instances = {}
    _lock = threading.Lock()

    _JOINT_NUMS = 6
    _JOINT_INDEX_LIST = [i for i in range(1, _JOINT_NUMS + 1)] + [255]

    _Parser = ProtocolParserInterface

    class EFFECTOR:
        """
        End-effector kind constants.

        Use:
            robot.init_effector(robot.EFFECTOR.AGX_GRIPPER)
        """

        AGX_GRIPPER: Final[Literal["agx_gripper"]] = "agx_gripper"
        REVO2: Final[Literal["revo2"]] = "revo2"

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        cls._JOINT_INDEX_LIST = [i for i in range(1, cls._JOINT_NUMS + 1)] + [255]

    def __init__(self, config: dict):
        self._config = config.copy()
        self._ctx = DriverContext(config)
        self._connected = False
        self._effector_kind: Optional[str] = None
        self._effector = None
        self._parser = self._Parser(self._ctx.fps)
        self._arm_ctx = ArmDriverContext(config, self._ctx, self._parser)

        # TCP
        self._tcp_offset_pose: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._T_f_t = None
        self._T_t_f = None

    def _send_msg(self, msg: AttributeBase) -> None:
        """Send one control message.

        Parameters
        ----------
        `msg`: AttributeBase
        - The message to send.
        """
        if isinstance(msg, AttributeBase):
            data = self._parser.pack(msg)
            if data is not None:
                self._ctx.get_comm().send(data)
        else:
            raise TypeError(
                "msg must be AttributeBase"
            )

    def _send_msgs(
        self,
        msgs: List[AttributeBase]
    ) -> None:
        """Send a sequence of control messages (with optional intervals).

        Parameters
        ----------
        `msgs`: list[AttributeBase]
        - The messages to send.
        """
        if isinstance(msgs, list):
            for i, msg in enumerate(msgs):
                self._send_msg(msg)
        else:
            raise TypeError(
                "msgs must be a list of AttributeBase"
            )

    @property
    def joint_nums(self):
        return self._JOINT_NUMS
    
    def get_context(self):
        return self._ctx

    @overload
    def init_effector(
        self, effector: None
    ) -> None:
        """Initialize end-effector driver exactly once and return the driver instance.

        Notes
        -----
        - This method is intentionally one-shot to avoid registering multiple
          callbacks into the same DriverContext threads.
        - If called again, it raises RuntimeError.
        """
        ...

    @overload
    def init_effector(
        self, effector: Literal["agx_gripper"]
    ) -> "AgxGripperDriverDefault":
        """agx_gripper end-effector driver.
        """
        ...

    @overload
    def init_effector(
        self, effector: Literal["revo2"]
    ) -> "Revo2DriverDefault":
        """revo2 end-effector driver.
        """
        ...

    def init_effector(self, effector: str):
        """Initialize end-effector driver exactly once and return the driver instance.
        """
        if self._effector_kind is not None:
            raise RuntimeError(
                f"effector already initialized: {self._effector_kind}. "
                "Create a new robotic arm instance if you need a different effector."
            )

        effector_kind = str(effector).strip().lower()
        self._effector_kind = effector_kind

        if effector_kind == self.EFFECTOR.AGX_GRIPPER:
            from ..effector.agx_gripper import AgxGripperDriverDefault

            self._effector = AgxGripperDriverDefault(self._config, self.get_context())
            return self._effector

        if effector_kind == self.EFFECTOR.REVO2:
            from ..effector.revo2 import Revo2DriverDefault

            self._effector = Revo2DriverDefault(self._config, self.get_context())
            return self._effector

        raise ValueError(f"Unsupported effector kind: {effector}")

    def get_driver_version(self):
        raise NotImplementedError

    def create_comm(self, config: dict = {}, comm: str = "can"):
        return self._ctx.create_comm(config, comm)

    def connect(self, start_read_thread: bool = True) -> None:
        if not self._ctx.get_comm():
            self._ctx.init_comm()
        if self._ctx.get_comm() is None:
            raise ValueError("comm is None")
        with self._lock:
            if self._connected:
                return
            self._connected = self._ctx.get_comm().is_connected()
        if start_read_thread:
            self._ctx.start_th()

    def is_connected(self) -> bool:
        return self._ctx.get_comm().is_connected()

    def is_ok(self):
        return self._arm_ctx.is_ok()

    def get_fps(self):
        return self._arm_ctx.get_fps()

    def get_config(self) -> dict:
        return self._config

    def get_type(self):
        return self._ctx.get_comm().get_type()

    def get_channel(self):
        return self._ctx.get_comm().get_channel()

    def get_joint_states(self):
        raise NotImplementedError

    def get_flange_pose(self):
        raise NotImplementedError

    def get_arm_status(self):
        raise NotImplementedError

    def get_driver_states(self):
        raise NotImplementedError

    def get_motor_states(self):
        raise NotImplementedError

    def enable(self):
        raise NotImplementedError

    def disable(self):
        raise NotImplementedError

    def reset(self):
        raise NotImplementedError

    def electronic_emergency_stop(self):
        raise NotImplementedError

    # -------------------------- TCP --------------------------

    def set_tcp_offset(self, pose: List[float]):
        """Set TCP offset pose in the flange frame.

        Parameters
        ----------
        `pose`: list[float]
        - `[x, y, z, roll, pitch, yaw]` in flange frame.
        - `x, y, z`: meters.
        - `roll, pitch, yaw`: radians (Euler angles around `X/Y/Z`).
          - `roll`, `yaw` must be within `[-pi, pi]`
          - `pitch` must be within `[-pi/2, pi/2]`
        """
        Validator.validate_pose6(
            pose,
            name="set_tcp_offset",
            validate_angle_limits=True
        )
        self._tcp_offset_pose = pose
        if all(x == 0.0 for x in self._tcp_offset_pose):
            self._T_f_t = None
            self._T_t_f = None
        else:
            self._T_f_t = pose6_to_T(self._tcp_offset_pose)
            self._T_t_f = inv_T(self._T_f_t)

    def get_tcp_pose(self):
        """Get TCP pose by applying the configured TCP offset to the flange pose.

        Returns
        -------
        MessageAbstract[list[float]] | None
            `msg`: `[x, y, z, roll, pitch, yaw]` (TCP pose in base frame)
        """
        flange: Optional[MessageAbstract] = self.get_flange_pose()
        if flange is None:
            return None
        
        return MessageAbstract(
            msg_type="tcp_pose",
            msg=self.get_flange2tcp_pose(flange.msg),
            timestamp=flange.timestamp,
            hz=flange.hz,
        )

    def get_flange2tcp_pose(self, flange_pose: List[float]):
        """Convert a flange pose (base frame) to the corresponding TCP pose.

        Parameters
        ----------
        `flange_pose`: `[x, y, z, roll, pitch, yaw]`
        - `x, y, z`: meters.
        - `roll, pitch, yaw`: radians (Euler angles around `X/Y/Z`).
          - `roll`, `yaw` must be within `[-pi, pi]`
          - `pitch` must be within `[-pi/2, pi/2]`

        Returns
        -------
        `tcp_pose`: `[x, y, z, roll, pitch, yaw]`
        - `x, y, z`: meters.
        - `roll, pitch, yaw`: radians (Euler angles around `X/Y/Z`).
          - `roll`, `yaw` must be within `[-pi, pi]`
          - `pitch` must be within `[-pi/2, pi/2]`

        Examples
        --------
        >>> robot.set_tcp_offset([0, 0, 0.1, 0, 0, 0])
        >>> flange_pose = robot.get_flange_pose()
        >>> if flange_pose is not None:
        >>>     print(robot.get_flange2tcp_pose(flange_pose.msg))
        """
        Validator.validate_pose6(
            flange_pose,
            name="flange2tcp_pose",
            validate_angle_limits=True
        )
        if self._T_f_t is None:
            return flange_pose
        
        T_w_f = pose6_to_T(flange_pose)
        T_w_t = matmul4(T_w_f, self._T_f_t)
        return T_to_pose6(T_w_t)

    def get_tcp2flange_pose(self, tcp_pose: List[float]):
        """Convert a target TCP pose (base frame) to the corresponding flange pose.

        Notes
        -----
        If you call:
            `flange_pose = robot.get_tcp2flange_pose(target_tcp_pose)`

            `robot.move_p(flange_pose)`

        the TCP will move to `target_tcp_pose` (subject to kinematics and controller).
        """
        Validator.validate_pose6(
            tcp_pose,
            name="tcp2flange_pose",
            validate_angle_limits=True
        )
        if self._T_t_f is None:
            return tcp_pose
        
        T_w_t = pose6_to_T(tcp_pose)
        T_w_f = matmul4(T_w_t, self._T_t_f)
        return T_to_pose6(T_w_f)

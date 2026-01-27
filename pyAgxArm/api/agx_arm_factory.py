import copy
import inspect
from typing import Type, Dict, TypeVar
from typing_extensions import Literal
from .constants import ROBOT_OPTION_FIELDS, ROBOT_JOINT_LIMIT_PRESET, ROBOT_JOINT_NAME
from ..protocols.can_protocol.comms import *
from ..protocols.can_protocol.drivers.piper import PiperDriverDefault
from ..protocols.can_protocol.drivers.nero import NeroDriverDefault
from ..protocols.can_protocol.drivers.piper_h import PiperHDriverDefault
from ..protocols.can_protocol.drivers.piper_l import PiperLDriverDefault
from ..protocols.can_protocol.drivers.piper_x import PiperXDriverDefault


def extract_kwargs(func, source: dict) -> dict:
    sig = inspect.signature(func)
    return {
        k: source[k]
        for k in sig.parameters.keys()
        if k in source
    }


def create_agx_arm_config(
        robot: Literal["nero", "piper", "piper_h", "piper_l", "piper_x"],
        comm: Literal["can"] = "can",
        firmeware_version: str = "default",
        **kwargs):
    config = {
        "robot": robot,
        "firmeware_version": firmeware_version,
        "log": {
            "level": kwargs.get("log_level", "INFO"),
            "path": kwargs.get("log_path", ""),
        },
    }

    # ---------- robot-specific options ----------
    allowed_fields = ROBOT_OPTION_FIELDS.get(robot, set())

    for field in allowed_fields:
        if field in kwargs:
            config[field] = kwargs[field]
    # ---------- joint name ----------
    config["joint_names"] = ROBOT_JOINT_NAME.get(robot)
    # ---------- joint limit ----------
    preset_joint_limits = ROBOT_JOINT_LIMIT_PRESET.get(robot)
    if preset_joint_limits is None:
        raise ValueError(f"No joint limit preset for robot={robot}")

    # 使用深拷贝，避免污染全局 preset
    final_joint_limits = copy.deepcopy(preset_joint_limits)

    user_joint_limits = kwargs.get("joint_limits")
    if user_joint_limits is not None:
        if not isinstance(user_joint_limits, dict):
            raise TypeError("joint_limits must be a dict")

        for joint, limit in user_joint_limits.items():
            if joint not in final_joint_limits:
                raise ValueError(f"Invalid joint name: {joint}")
            if not (isinstance(limit, (list, tuple)) and len(limit) == 2):
                raise ValueError(f"Invalid limit format for {joint}")
            final_joint_limits[joint] = list(limit)

    config["joint_limits"] = final_joint_limits
    # ---------- comm ----------
    if comm == "can":
        config["comm"] = {
            "type": "can",
            "can": create_can_comm_config(
                    **extract_kwargs(create_can_comm_config, kwargs)
            ),
        }
    else:
        raise ValueError(f"Unsupported comm type: {comm}")

    return config


T = TypeVar("T")


class AgxArmFactory:

    _registry: Dict[str, Dict[str, Dict[str, Type]]] = {
        "piper": {
            "can": {
                "default": PiperDriverDefault,
            },
        },
        "nero": {
            "can": {
                "default": NeroDriverDefault,
            },
        },
        "piper_h": {
            "can": {
                "default": PiperHDriverDefault,
            },
        },
        "piper_l": {
            "can": {
                "default": PiperLDriverDefault,
            },
        },
        "piper_x": {
            "can": {
                "default": PiperXDriverDefault,
            },
        },
    }

    # -------------------------------------------------
    @classmethod
    def register_arm(
        cls,
        *,
        robot: str,
        comm: str,
        firmeware_version: str,
        driver_cls: Type,
    ) -> None:
        """
        注册 Driver

        robot   : piper / nero
        comm    : can
        firmeware_version : default / ...
        """
        cls._registry.setdefault(robot, {})
        cls._registry[robot].setdefault(comm, {})
        cls._registry[robot][comm][firmeware_version] = driver_cls

    # -------------------------------------------------
    @classmethod
    def load_class(cls, config: dict) -> Type:
        """
        根据 config 获取 Driver 类（不实例化）
        """
        robot = config["robot"]
        comm = config["comm"]["type"]
        firmeware_version = config.get("firmeware_version", "default")

        try:
            return cls._registry[robot][comm][firmeware_version]
        except KeyError as e:
            raise KeyError(
                f"Driver not registered: robot={robot}, comm={comm}, version={firmeware_version}"
            ) from e

    # -------------------------------------------------
    @classmethod
    def create_arm(cls, config: dict, **kwargs) -> T:
        """
        Create a robotic arm Driver instance.
        """
        arm_cls: Type[T] = cls.load_class(config)
        return arm_cls(config=config, **kwargs)

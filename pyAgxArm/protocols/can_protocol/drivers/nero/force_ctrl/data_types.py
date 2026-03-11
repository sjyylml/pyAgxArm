from dataclasses import dataclass, field
import numpy as np


@dataclass
class JointCommands:
    """Thread-safe command buffer for MIT control."""
    pos: np.ndarray
    vel: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    torques: np.ndarray

    @classmethod
    def zeros(cls, n_joints: int = 7) -> "JointCommands":
        return cls(
            pos=np.zeros(n_joints),
            vel=np.zeros(n_joints),
            kp=np.zeros(n_joints),
            kd=np.zeros(n_joints),
            torques=np.zeros(n_joints),
        )


@dataclass
class JointState:
    """Current joint state feedback."""
    pos: np.ndarray
    vel: np.ndarray
    torque: np.ndarray
    timestamp: float = 0.0

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, TypeVar, runtime_checkable, Union
from datetime import datetime, UTC

T = TypeVar("T", bound="MsgProtocol")

@runtime_checkable
class MsgProtocol(Protocol):
    @classmethod
    def routing_key(cls) -> str:
        ...

class CtrlMsg:
    type : str 
    @classmethod
    def routing_key(cls) -> str:
        return "robotarm.ctrl"

@dataclass
class LoadProgram(CtrlMsg):
    joint_positions: list[int]
    max_velocity: float
    acceleration: float
    type : str  = "load_program"

@dataclass
class Play(CtrlMsg):
    type : str = "play"

@dataclass
class Pause(CtrlMsg):
    type : str  = "pause"

@dataclass
class Stop(CtrlMsg):
    type : str = "stop"

@dataclass
class Wear:
    duration: float
    joints: list[int]

    @property
    def id(self) -> str:
        return "wear"


@dataclass
class StuckJoint:
    joints: list[int]

    @property
    def id(self) -> str:
        return "stuck_joint"


@dataclass
class InjectFault(CtrlMsg):
    fault_type: Union[Wear, StuckJoint]
    type : str = "inject_fault"

@dataclass(kw_only=True)
class TimeStamped:
    timestamp: str = field(
        default_factory=lambda: datetime.now(UTC).isoformat()
    )

@dataclass
class RobotStateMessage(TimeStamped):
    robot_mode: str
    q_actual: list[float]
    qd_actual: list[float]
    q_target: list[float]
    joint_max_speed: list[float]
    joint_max_acceleration: list[float]
    tcp_pose: list[float]

@dataclass
class PhysicalTwinState(RobotStateMessage):
    @classmethod
    def routing_key(cls) -> str:
        return "robotarm.pt.state"

@dataclass
class KinematicModelState(RobotStateMessage):
    @classmethod
    def routing_key(cls) -> str:
        return "rt_model.dt.state"

@dataclass
class Deviation(TimeStamped):
    joint_deviations: list[float]

    @classmethod
    def routing_key(cls) -> str:
        return "deviation.status"


@dataclass
class StuckJointStatus(TimeStamped):
    stuck_joints: list[bool]

    @classmethod
    def routing_key(cls) -> str:
        return "stuck_joint.status"


@dataclass
class FilteredState(TimeStamped):
    q_filtered: list[float]
    qd_filtered: list[float]

    @classmethod
    def routing_key(cls) -> str:
        return "filtered.state"
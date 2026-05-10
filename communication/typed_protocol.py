from __future__ import annotations

from dataclasses import dataclass, field, asdict
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
    joint_positions: list[float]
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

@dataclass(kw_only=True)
class InjectFault(CtrlMsg):
    type : str = "inject_fault"

@dataclass
class InjectWear(InjectFault):
    duration: float
    joints: list[int]
    fault_type: str = "wear"

@dataclass
class InjectStuckJoint(InjectFault):
    joints: list[int]
    fault_type: str = "stuck_joint"


@dataclass(kw_only=True)
class TimeStamped:
    timestamp: float = field(
        default_factory=lambda: datetime.now(UTC).timestamp()
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
class FilteredState(RobotStateMessage):
    @classmethod
    def routing_key(cls) -> str:
        return "filtered.state"

@dataclass
class Deviation(TimeStamped):
    joint_deviations: list[float]

    @classmethod
    def routing_key(cls) -> str:
        return "deviation.status"


@dataclass
class StuckJointStatus(TimeStamped):
    stuck_joints: list[bool]
    joint_positions : list[float]

    @classmethod
    def routing_key(cls) -> str:
        return "stuck_joint.status"


@dataclass
class LoadTCPProgram(TimeStamped):
    tcp_position: list[float]
    tcp_rotation: list[float]
    max_velocity: float
    acceleration: float

    @classmethod
    def routing_key(cls) -> str:
        return "load_program.tcp"


@dataclass
class WearStatus(TimeStamped):
    wear_detected: bool
    affected_joints: list[int]

    @classmethod
    def routing_key(cls) -> str:
        return "wear.status"

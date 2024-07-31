from dataclasses import dataclass, field
from typing import Optional
from protoclass.ADCTrajectory import GearPosition
from enum import Enum
from protoclass.Pose import Pose

@dataclass
class VehicleState:
    """
    VehicleState class, oriented from protobuf message
    """

    class DrivingMode(Enum):

        COMPLETE_MANUAL = 0       # human drive
        COMPLETE_AUTO_DRIVE = 1
        AUTO_STEER_ONLY = 2       # only steer
        AUTO_SPEED_ONLY = 3       # include throttle and brake
        EMERGENCY_MODE = 4        # security mode when manual intervention happens, only response status

    x: float = field(default=0.0)
    y: float = field(default=0.0)
    z: float = field(default=0.0)
    timestamp: float = field(default=0.0)
    roll: float = field(default=0.0)
    pitch: float = field(default=0.0)
    yaw: float = field(default=0.0)
    heading: float = field(default=0.0)
    kappa: float = field(default=0.0)
    linear_velocity: float = field(default=0.0)
    angular_velocity: float = field(default=0.0)
    linear_acceleration: float = field(default=0.0)
    gear: Optional[GearPosition] = None
    driving_mode: Optional[DrivingMode] = None
    pose: Optional[Pose] = None
    steering_percentage: Optional[float] = None

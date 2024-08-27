from dataclasses import dataclass
from typing import Optional, Union, Any
from protoclass.Header import Header
from protoclass.routing import RoutingResponse

@dataclass
class ParkingCommand:
    """
    ParkingCommand class, oriented from protobuf message
    """

    header: Optional[Header] = None
    parking_spot_id: str = ''
    """The id of the parking spot on the map."""
    target_speed: Optional[float] = None
    """
    Expected speed when executing this command. If "target_speed" > maximum
    speed of the vehicle, use maximum speed of the vehicle instead. If it is
    not given, the default target speed of system will be used.
    """

@dataclass
class PlanningCommand:
    """
    PlanningCommand class, oriented from protobuf message
    """

    header: Optional[Header] = None
    command_id: Optional[int] = -1
    """Unique identification for command."""
    lane_follow_command: Optional[RoutingResponse] = None
    """Move along the lanes on map."""
    target_speed: Optional[float] = None
    """Target speed in command."""
    is_motion_command: Optional[bool] = False
    """Indicate if the command is a motion command."""
    command: Optional[Union[ParkingCommand,   # Move to the given pose with open space planning trajectory.
                            Any]              # Custom command defined by user for extensibility.
                     ] = None

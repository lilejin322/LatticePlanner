from enum import Enum
from dataclasses import dataclass
from typing import List, Optional
from protoclass.Header import Header

@dataclass
class TrafficLight:
    """
    TrafficLight class, oriented from protobuf message
    """

    class Color(Enum):

        UNKNOWN = 0
        RED = 1
        YELLOW = 2
        GREEN = 3
        BLACK = 4

    color: Optional[Color] = None
    id: Optional[str] = None
    """Traffic light string-ID in the map data"""
    confidence: Optional[float] = 1.0
    """Confidence about the detected results, between 0 and 1"""
    tracking_time: Optional[float] = None
    """Duration of the traffic light since detected"""
    blink: Optional[bool] = None
    """Is the traffic light blinking"""
    remaining_time: Optional[float] = None
    """v2x traffic light remaining time"""

@dataclass
class TrafficLightBox:
    """
    TrafficLightBox class, oriented from protobuf message
    """

    x: Optional[int] = None
    y: Optional[int] = None
    width: Optional[int] = None
    height: Optional[int] = None
    color: Optional[TrafficLight.Color] = None
    selected: Optional[bool] = None
    camera_name: Optional[str] = None

@dataclass
class TrafficLightDebug:
    """
    TrafficLightDebug class, oriented from protobuf message
    """

    cropbox: Optional[TrafficLightBox] = None
    box: List[TrafficLightBox] = None
    signal_num: Optional[int] = None
    valid_pos: Optional[int] = None
    ts_diff_pos: Optional[float] = None
    ts_diff_sys: Optional[float] = None
    project_error: Optional[int] = None
    distance_to_stop_line: Optional[float] = None
    crop_roi: List[TrafficLightBox] = None
    projected_roi: List[TrafficLightBox] = None
    rectified_roi: List[TrafficLightBox] = None
    debug_roi: List[TrafficLightBox] = None

@dataclass
class TrafficLightDetection:
    """
    TrafficLightDetection class, oriented from protobuf message
    """

    class CameraID(Enum):

        CAMERA_FRONT_LONG = 0
        CAMERA_FRONT_NARROW = 1
        CAMERA_FRONT_SHORT = 2
        CAMERA_FRONT_WIDE = 3

    header: Optional[Header] = None
    traffic_light: List[TrafficLight] = None
    traffic_light_debug: Optional[TrafficLightDebug] = None
    contain_lights: Optional[bool] = None
    camera_id: Optional[CameraID] = None
    camera_name: Optional[str] = None

from dataclasses import dataclass
from typing import Optional
from enum import Enum
from protoclass.Header import Header
import math

@dataclass
class CloseToCrosswalk:
    """
    CloseToCrosswalk class, oriented from protobuf message
    """

    id: Optional[str] = None
    distance: Optional[float] = math.nan

@dataclass
class CloseToClearArea:
    """
    CloseToClearArea class, oriented from protobuf message
    """

    id: Optional[str] = None
    distance: Optional[float] = math.nan

@dataclass
class CloseToJunction:
    """
    CloseToJunction class, oriented from protobuf message
    """

    class JunctionType(Enum):

        PNC_JUNCTION = 1
        JUNCTION = 2

    id: Optional[str] = None
    type: Optional[JunctionType] = None
    distance: Optional[float] = math.nan

@dataclass
class CloseToSignal:
    """
    CloseToSignal class, oriented from protobuf message
    """

    id: Optional[str] = None
    distance: Optional[float] = math.nan

@dataclass
class CloseToStopSign:
    """
    CloseToStopSign class, oriented from protobuf message
    """

    id: Optional[str] = None
    distance: Optional[float] = math.nan

@dataclass
class CloseToYieldSign:
    """
    CloseToYieldSign class, oriented from protobuf message
    """

    id: Optional[str] = None
    distance: Optional[float] = math.nan

@dataclass
class Stories:
    """
    Stories class, oriented from protobuf message
    
    Usage guide for action modules:
    1. Call `stories.has_XXX()` to check if a story you are interested is in charge.
    2. Access the story details if necessary, and take action accordingly.
    """

    header: Optional[Header] = None
    close_to_clear_area: Optional[CloseToClearArea] = None
    close_to_crosswalk: Optional[CloseToCrosswalk] = None
    close_to_junction: Optional[CloseToJunction] = None
    close_to_signal: Optional[CloseToSignal] = None
    close_to_stop_sign: Optional[CloseToStopSign] = None
    close_to_yield_sign: Optional[CloseToYieldSign] = None

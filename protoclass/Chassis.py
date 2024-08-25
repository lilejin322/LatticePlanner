from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Any
import math
from protoclass.Header import Header
from protoclass.DecisionResult import VehicleSignal
from protoclass.ADCTrajectory import EngageAdvice, Point3D
from protoclass.Pose import Quaternion

@dataclass
class WheelSpeed:
    """
    WheelSpeed class, oriented from protobuf message
    """

    class WheelSpeedType(Enum):

        FORWARD = 0
        BACKWARD = 1
        STANDSTILL = 2
        INVALID = 3

    is_wheel_spd_rr_valid: Optional[bool] = field(default=False)
    wheel_direction_rr: Optional[WheelSpeedType] = WheelSpeedType.INVALID
    wheel_spd_rr: Optional[float] = field(default=0.0)
    is_wheel_spd_rl_valid: Optional[bool] = field(default=False)
    wheel_direction_rl: Optional[WheelSpeedType] = WheelSpeedType.INVALID
    wheel_spd_rl: Optional[float] = field(default=0.0)
    is_wheel_spd_fr_valid: Optional[bool] = field(default=False)
    wheel_direction_fr: Optional[WheelSpeedType] = WheelSpeedType.INVALID
    wheel_spd_fr: Optional[float] = field(default=0.0)
    is_wheel_spd_fl_valid: Optional[bool] = field(default=False)
    wheel_direction_fl: Optional[WheelSpeedType] = WheelSpeedType.INVALID
    wheel_spd_fl: Optional[float] = field(default=0.0)

@dataclass
class Sonar:
    """
    Sonar class, oriented from protobuf message
    """

    range: Optional[float] = None              # Meter
    translation: Optional[Point3D] = None      # Meter
    rotation: Optional[Quaternion] = None

@dataclass
class Surround:
    """
    Surround class, oriented from protobuf message
    """

    cross_traffic_alert_left: Optional[bool] = None
    cross_traffic_alert_left_enabled: Optional[bool] = None
    blind_spot_left_alert: Optional[bool] = None
    blind_spot_left_alert_enabled: Optional[bool] = None
    cross_traffic_alert_right: Optional[bool] = None
    cross_traffic_alert_right_enabled: Optional[bool] = None
    blind_spot_right_alert: Optional[bool] = None
    blind_spot_right_alert_enabled: Optional[bool] = None
    sonar00: Optional[float] = None
    sonar01: Optional[float] = None
    sonar02: Optional[float] = None
    sonar03: Optional[float] = None
    sonar04: Optional[float] = None
    sonar05: Optional[float] = None
    sonar06: Optional[float] = None
    sonar07: Optional[float] = None
    sonar08: Optional[float] = None
    sonar09: Optional[float] = None
    sonar10: Optional[float] = None
    sonar11: Optional[float] = None
    sonar_enabled: Optional[bool] = None
    sonar_fault: Optional[bool] = None
    sonar_range: Optional[List[float]] = None
    sonar: Optional[List[Sonar]] = None

class GpsQuality(Enum):

    FIX_NO = 0
    FIX_2D = 1
    FIX_3D = 2
    FIX_INVALID = 3

@dataclass
class ChassisGPS:
    """
    ChassisGPS class, oriented from protobuf message
    """

    latitude: Optional[float] = None
    longitude: Optional[float] = None
    gps_valid: Optional[bool] = None

    year: Optional[int] = None
    month: Optional[int] = None
    day: Optional[int] = None
    hours: Optional[int] = None
    minutes: Optional[int] = None
    seconds: Optional[int] = None
    compass_direction: Optional[float] = None
    pdop: Optional[float] = None
    is_gps_fault: Optional[bool] = None
    is_inferred: Optional[bool] = None

    altitude: Optional[float] = None
    heading: Optional[float] = None
    hdop: Optional[float] = None
    vdop: Optional[float] = None
    quality: Optional[GpsQuality] = None
    num_satellites: Optional[int] = None
    gps_speed: Optional[float] = None

@dataclass
class VehicleID:

    vin: Optional[str] = None
    plate: Optional[str] = None
    other_unique_id: Optional[str] = None

@dataclass
class CheckResponse:
    """
    CheckResponseSignal, oriented from protobuf message
    """

    is_eps_online: Optional[bool] = field(default=False)
    is_epb_online: Optional[bool] = field(default=False)
    is_esp_online: Optional[bool] = field(default=False)
    is_vtog_online: Optional[bool] = field(default=False)
    is_scu_online: Optional[bool] = field(default=False)
    is_switch_online: Optional[bool] = field(default=False)
    is_vcu_online: Optional[bool] = field(default=False)

@dataclass
class Chassis:
    """
    Chassis class, oriented from protobuf message
    """

    class DrivingMode(Enum):

        COMPLETE_MANUAL = 0
        """human drive"""
        COMPLETE_AUTO_DRIVE = 1
        AUTO_STEER_ONLY = 2
        """only steer"""
        AUTO_SPEED_ONLY = 3
        """include throttle and brake"""
        EMERGENCY_MODE = 4
        """security mode when manual intervention happens, only response status"""

    class ErrorCode(Enum):

        NO_ERROR = 0
        CMD_NOT_IN_PERIOD = 1
        """control cmd not in period"""
        CHASSIS_ERROR = 2
        """car chassis report error, like steer, brake, throttle, gear fault"""
        MANUAL_INTERVENTION = 3
        """human manual intervention"""
        CHASSIS_CAN_NOT_IN_PERIOD = 4
        """receive car chassis can frame not in period"""
        UNKNOWN_ERROR = 5
        """classify the types of the car chassis errors"""
        CHASSIS_ERROR_ON_STEER = 6
        CHASSIS_ERROR_ON_BRAKE = 7
        CHASSIS_ERROR_ON_THROTTLE = 8
        CHASSIS_ERROR_ON_GEAR = 9
        CHASSIS_CAN_LOST = 10

    class GearPosition(Enum):

        GEAR_NEUTRAL = 0
        GEAR_DRIVE = 1
        GEAR_REVERSE = 2
        GEAR_PARKING = 3
        GEAR_LOW = 4
        GEAR_INVALID = 5
        GEAR_NONE = 6

    class BumperEvent(Enum):

        BUMPER_INVALID = 0
        BUMPER_NORMAL = 1
        BUMPER_PRESSED = 2

    engine_started: Optional[bool] = None

    engine_rpm: Optional[float] = field(default_factory=lambda: math.nan)
    """Engine speed in RPM"""

    speed_mps: Optional[float] = field(default_factory=lambda: math.nan)
    """Vehicle Speed in meters per second"""

    odometer_m: Optional[float] = None
    """Vehicle odometer in meters"""

    fuel_range_m: Optional[int] = None
    """Fuel range in meters"""

    throttle_percentage: Optional[float] = field(default_factory=lambda: math.nan)
    """Real throttle location in [%], ranging from 0 to 100"""

    brake_percentage: Optional[float] = field(default_factory=lambda: math.nan)
    """Real brake location in [%], ranging from 0 to 100"""

    steering_percentage: Optional[float] = field(default_factory=lambda: math.nan)
    """
    Real steering location in [%], ranging from -100 to 100
    steering_angle / max_steering_angle
    Clockwise: negative
    CountClockwise: positive
    """

    steering_torque_nm: Optional[float] = field(default_factory=lambda: math.nan)
    """Applied steering torque in [Nm]"""

    parking_brake: Optional[bool] = None
    """Parking brake status"""

    wiper: Optional[bool] = None
    driving_mode: Optional[DrivingMode] = DrivingMode.COMPLETE_MANUAL
    error_code: Optional[ErrorCode] = ErrorCode.NO_ERROR
    gear_location: Optional[GearPosition] = None

    steering_timestamp: Optional[float] = None
    """Timestamp for steering module (in seconds, with 1e-6 accuracy)"""

    # chassis also needs it own sending timestamp
    header: Optional[Header] = None

    chassis_error_mask: Optional[int] = 0

    signal: Optional[VehicleSignal] = None

    chassis_gps: Optional[ChassisGPS] = None
    """Only available for Lincoln now"""

    engage_advice: Optional[EngageAdvice] = None

    wheel_speed: Optional[WheelSpeed] = None

    surround: Optional[Surround] = None

    vehicle_id: Optional[VehicleID] = None

    battery_soc_percentage: Optional[int] = -1

    throttle_percentage_cmd: Optional[float] = field(default_factory=lambda: math.nan)
    """Real send throttle location in [%], ranging from 0 to 100"""

    brake_percentage_cmd: Optional[float] = field(default_factory=lambda: math.nan)
    """Real send brake location in [%], ranging from 0 to 100"""

    steering_percentage_cmd: Optional[float] = field(default_factory=lambda: math.nan)
    """
    Real send steering location in [%], ranging from -100 to 100
    steering_angle / max_steering_angle
    Clockwise: negative
    CountClockwise: positive
    """

    front_bumper_event: Optional[BumperEvent] = BumperEvent.BUMPER_INVALID

    back_bumper_event: Optional[BumperEvent] = BumperEvent.BUMPER_INVALID

    check_response: Optional[CheckResponse] = None

    custom_status: Optional[Any] = None
    """Custom chassis operation command defined by user for extensibility"""

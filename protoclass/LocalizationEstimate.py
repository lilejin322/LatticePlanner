from dataclasses import dataclass
from typing import List, Optional
from protoclass.Header import Header
from protoclass.Pose import Pose
from protoclass.ADCTrajectory import Point3D
from protoclass.TrajectoryPoint import TrajectoryPoint
from enum import Enum

@dataclass
class Uncertainty:
    """
    Uncertainty class, oriented from protobuf message
    """

    position_std_dev: Optional[Point3D] = None
    """Standard deviation of position, east/north/up in meters."""

    orientation_std_dev: Optional[Point3D] = None
    """Standard deviation of quaternion qx/qy/qz, unitless."""

    linear_velocity_std_dev: Optional[Point3D] = None
    """Standard deviation of linear velocity, east/north/up in meters per second."""

    linear_acceleration_std_dev: Optional[Point3D] = None
    """Standard deviation of linear acceleration, right/forward/up in meters per square second."""

    angular_velocity_std_dev: Optional[Point3D] = None
    """Standard deviation of angular velocity, right/forward/up in radians per second."""

    # TODO: Define covariance items when needed.

class LocalLidarConsistency(Enum):
    """
    LiDAR-based localization result check (the difference between lidar and sins result)
    """

    MSF_LOCAL_LIDAR_CONSISTENCY_00 = 0
    """The difference is less than threshold 1"""
    MSF_LOCAL_LIDAR_CONSISTENCY_01 = 1
    """The difference is bigger than threshold 1 but less than threshold 2"""
    MSF_LOCAL_LIDAR_CONSISTENCY_02 = 2
    """The difference is bigger than threshold 2"""
    MSF_LOCAL_LIDAR_CONSISTENCY_03 = 3
    """others"""

class GnssConsistency(Enum):

    MSF_GNSS_CONSISTENCY_00 = 0
    """The difference is less than threshold 1"""
    MSF_GNSS_CONSISTENCY_01 = 1
    """The difference is bigger than threshold 1 but less than threshold 2"""
    MSF_GNSS_CONSISTENCY_02 = 2
    """The difference is bigger than threshold 2"""
    MSF_GNSS_CONSISTENCY_03 = 3
    """others"""

class LocalLidarStatus(Enum):
    """
    LiDAR-based loclaization module status
    """

    MSF_LOCAL_LIDAR_NORMAL = 0
    """Localization result satisfy threshold"""
    MSF_LOCAL_LIDAR_MAP_MISSING = 1
    """Can't find localization map (config.xml)"""
    MSF_LOCAL_LIDAR_EXTRINSICS_MISSING = 2
    """Missing extrinsic parameters"""
    MSF_LOCAL_LIDAR_MAP_LOADING_FAILED = 3
    """Fail to load localization map"""
    MSF_LOCAL_LIDAR_NO_OUTPUT = 4
    """No output (comparing to timestamp of imu msg)"""
    MSF_LOCAL_LIDAR_OUT_OF_MAP = 5
    """Coverage of online pointcloud and map is lower than threshold"""
    MSF_LOCAL_LIDAR_NOT_GOOD = 6
    """Localization result do not meet threshold"""
    MSF_LOCAL_LIDAR_UNDEFINED_STATUS = 7
    """others"""

class GnssPositionType(Enum):

    NONE = 0
    """No solution"""
    FIXEDPOS = 1
    """Position has been fixed by the FIX POSITION command or by position averaging"""
    FIXEDHEIGHT = 2
    """Position has been fixed by the FIX HEIGHT, or FIX AUTO, command or by position averaging"""
    FLOATCONV = 4
    """Solution from floating point carrier phase ambiguities"""
    WIDELANE = 5
    """Solution from wide-lane ambiguities"""
    NARROWLANE = 6
    """Solution from narrow-lane ambiguities"""
    DOPPLER_VELOCITY = 8
    """Velocity computed using instantaneous Doppler"""
    SINGLE = 16
    """Single point position"""
    PSRDIFF = 17
    """Pseudorange differential solution"""
    WAAS = 18
    """Solution calculated using corrections from an SBAS"""
    PROPOGATED = 19
    """Propagated by a Kalman filter without new observations"""
    OMNISTAR = 20
    """OmniSTAR VBS position"""
    L1_FLOAT = 32
    """Floating L1 ambiguity solution"""
    IONOFREE_FLOAT = 33
    """Floating ionospheric free ambiguity solution"""
    NARROW_FLOAT = 34
    """Floating narrow-lane ambiguity solution"""
    L1_INT = 48
    """Integer L1 ambiguity solution"""
    WIDE_INT = 49
    """Integer wide-lane ambiguity solution"""
    NARROW_INT = 50
    """Integer narrow-lane ambiguity solution"""
    RTK_DIRECT_INS = 51
    """RTK status where RTK filter is directly initialized from the INS filter"""
    INS_SBAS = 52
    """INS calculated position corrected for the antenna"""
    INS_PSRSP = 53
    """INS pseudorange single point solution - no DGPS corrections"""
    INS_PSRDIFF = 54
    """INS pseudorange differential solution"""   
    INS_RTKFLOAT = 55
    """INS RTK float point ambiguities solution"""
    INS_RTKFIXED = 56
    """INS RTK fixed ambiguities solution"""
    INS_OMNISTAR = 57
    """INS OmniSTAR VBS solution"""
    INS_OMNISTAR_HP = 58
    """INS OmniSTAR high precision solution"""
    INS_OMNISTAR_XP = 59
    """INS OmniSTAR extra precision solution"""
    OMNISTAR_HP = 64
    """OmniSTAR high precision"""
    OMNISTAR_XP = 65
    """OmniSTAR extra precision"""
    PPP_CONVERGING = 68
    """Precise Point Position(PPP) solution converging"""
    PPP = 69
    """Precise Point Position(PPP) solution"""
    INS_PPP_Converging = 73
    """INS NovAtel CORRECT Precise Point Position(PPP) solution converging"""
    INS_PPP = 74
    """INS NovAtel CORRECT Precise Point Position(PPP) solution"""
    MSG_LOSS = 91
    """Gnss position message loss"""

class MSFInitStatus(Enum):
    """
    The initializaiton status of localization module
    """

    MSF_INIT = 0
    MSF_ALIGNING = 1
    MSF_ALIGNED_INIT = 2
    MSF_ALIGNED_CONVERGED = 3
    MSF_ALIGNED_CONVERGING = 4
    MSF_ALIGNED_GOOD = 5
    MSF_ALIGNED_VALID = 6

class MsfRunningStatus(Enum):
    """
    The running status of localization module
    """

    MSF_SOL_LIDAR_GNSS = 0
    MSF_SOL_X_GNSS = 1
    MSF_SOL_LIDAR_X = 2
    MSF_SOL_LIDAR_XX = 3
    MSF_SOL_LIDAR_XXX = 4
    MSF_SOL_X_X = 5
    MSF_SOL_X_XX = 6
    MSF_SOL_X_XXX = 7
    MSF_SSOL_LIDAR_GNSS = 8
    MSF_SSOL_X_GNSS = 9
    MSF_SSOL_LIDAR_X = 10
    MSF_SSOL_LIDAR_XX = 11
    MSF_SSOL_LIDAR_XXX = 12
    MSF_SSOL_X_X = 13
    MSF_SSOL_X_XX = 14
    MSF_SSOL_X_XXX = 15
    MSF_NOSOL_LIDAR_GNSS = 16
    MSF_NOSOL_X_GNSS = 17
    MSF_NOSOL_LIDAR_X = 18
    MSF_NOSOL_LIDAR_XX = 19
    MSF_NOSOL_LIDAR_XXX = 20
    MSF_NOSOL_X_X = 21
    MSF_NOSOL_X_XX = 22
    MSF_NOSOL_X_XXX = 23
    MSF_RUNNING_INIT = 24

class MsfInitPoseSource(Enum):
    """
    The init pose source 
    """

    UNKNOWN_SOURCE = 0
    GNSS_HEADING = 1
    LOCAL_SEARCH_FROM_INTEGPVA = 2
    LOCAL_SEARCH_FROM_FILE = 3
    LOCAL_UPDATE_FROM_FILE = 4
    USER_INTERACTION = 5
    INSPVA_RECORD = 6
    GNSS_VELOCITY = 7
    GLOBAL_LIDAR = 8

class MsfInitPoseStatus(Enum):
    """
    The init pose status
    """

    INIT_WAITING = 0
    INIT_DOING = 1
    INIT_SUCCESSFUL = 2
    INIT_FAILED = 3
    INIT_TERMINATED = 4

@dataclass
class MsfInitDetails:
    """
    MsfInitDetails class, oriented from protobuf message
    The init details
    """

    init_pose_source: Optional[MsfInitPoseSource] = None
    local_search_from_integpva_status: Optional[MsfInitPoseStatus] = None
    local_search_from_file_status: Optional[MsfInitPoseStatus] = None
    local_update_from_file_status: Optional[MsfInitPoseStatus] = None
    user_interaction_status: Optional[MsfInitPoseStatus] = None
    gnss_position_type: Optional[int] = None
    local_from_global_lidar_status: Optional[MsfInitPoseStatus] = None

class LocalLidarQuality(Enum):

    MSF_LOCAL_LIDAR_VERY_GOOD = 0
    MSF_LOCAL_LIDAR_GOOD = 1
    MSF_LOCAL_LIDAR_NOT_BAD = 2
    MSF_LOCAL_LIDAR_BAD = 3

class MsfGnssMapOffsetStatus(Enum):
    """
    The status of gnss map offset
    """

    MSF_LOCAL_GNSS_MAP_OFFSET_NORMAL = 0
    MSF_LOCAL_GNSS_MAP_OFFSET_ABNORMAL = 1

@dataclass
class MsfGnssMapOffset:
    """
    The gnss map offset
    """

    status: Optional[MsfGnssMapOffsetStatus] = None
    offsetx: Optional[float] = None
    offsety: Optional[float] = None

@dataclass
class MsfStatus:
    """
    MsfStatus class, oriented from protobuf message
    The status of msf localization module
    """

    local_lidar_consistency: Optional[LocalLidarConsistency] = None
    gnss_consistency: Optional[GnssConsistency] = None
    local_lidar_status: Optional[LocalLidarStatus] = None
    gnsspos_position_type: Optional[GnssPositionType] = None
    heading_position_type: Optional[int] = None
    msf_init_status: Optional[MSFInitStatus] = None
    msf_running_status: Optional[MsfRunningStatus] = None
    msf_init_details: Optional[MsfInitDetails] = None
    local_lidar_quality: Optional[LocalLidarQuality] = None
    gnss_map_offset: Optional[MsfGnssMapOffset] = None
    lidar_alt_from_map: Optional[bool] = None
    local_lidar_score: Optional[float] = None
    local_reliability_status: Optional[bool] = False

class ImuMsgDelayStatus(Enum):
    """
    IMU msg status
    """

    IMU_DELAY_NORMAL = 0
    IMU_DELAY_1 = 1
    IMU_DELAY_2 = 2
    IMU_DELAY_3 = 3
    IMU_DELAY_ABNORMAL = 4

class ImuMsgMissingStatus(Enum):

    IMU_MISSING_NORMAL = 0
    IMU_MISSING_1 = 1
    IMU_MISSING_2 = 2
    IMU_MISSING_3 = 3
    IMU_MISSING_4 = 4
    IMU_MISSING_5 = 5
    IMU_MISSING_ABNORMAL = 6

class ImuMsgDataStatus(Enum):

    IMU_DATA_NORMAL = 0
    IMU_DATA_ABNORMAL = 1
    IMU_DATA_OTHER = 2

@dataclass
class MsfSensorMsgStatus:
    """
    The status of sensor msg, oriented from protobuf message
    """

    imu_delay_status: Optional[ImuMsgDelayStatus] = None
    imu_missing_status: Optional[ImuMsgMissingStatus] = None
    imu_data_status: Optional[ImuMsgDataStatus] = None

@dataclass
class LocalizationEstimate:
    """
    LocalizationEstimate class, oriented from protobuf message
    """

    header: Optional[Header] = None
    pose: Optional[Pose] = None
    uncertainty: Optional[Uncertainty] = None

    measurement_time: Optional[float] = None  # In seconds.
    """The time of pose measurement, seconds since 1970-1-1 (UNIX time)."""

    trajectory_point: List[TrajectoryPoint] = None
    """Future trajectory actually driven by the drivers"""

    msf_status: Optional[MsfStatus] = None
    """msf status"""

    sensor_status: Optional[MsfSensorMsgStatus] = None
    """msf quality"""

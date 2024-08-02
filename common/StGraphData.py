from protoclass.TrajectoryPoint import TrajectoryPoint
from common.SpeedLimit import SpeedLimit
from common.STBoundary import STBoundary
from typing import List,Tuple
from protoclass.STDrivableBoundary import STDrivableBoundary, STDrivableBoundaryInstance
from config import FLAGS_default_cruise_speed

kObsSpeedIgnoreThreshold: float = 100.0

class StGraphData:
    """
    StGraphData class
    """

    def __init__(self):
        self._init = False
        self._st_boundaries: List[STBoundary] = []
        self._min_s_on_st_boundaries: float = 0.0
        self._init_point: TrajectoryPoint = None
        self._speed_limit: SpeedLimit = None
        self._cruise_speed: float = 0.0
        self._path_data_length: float = 0.0
        self._path_length_by_conf: float = 0.0
        self._total_time_by_conf: float = 0.0
        self._st_drivable_boundary: STDrivableBoundary = None

    def LoadData(self, st_boundaries: List[STBoundary],
                 min_s_on_st_boundaries: float, init_point: TrajectoryPoint,
                 speed_limit: SpeedLimit, cruise_speed: float, path_data_length: float,
                 total_time_by_conf: float) -> None:

        self._init = True
        self._st_boundaries = st_boundaries
        self._min_s_on_st_boundaries = min_s_on_st_boundaries
        self._init_point = init_point
        self._speed_limit = speed_limit
        self._cruise_speed = cruise_speed
        self._path_data_length = path_data_length
        self._total_time_by_conf = total_time_by_conf

    def is_initialized(self) -> bool:
        """
        Check if the data is initialized

        :returns: True if the data is initialized, False otherwise
        :rtype: bool
        """

        return self._init

    @property
    def st_boundaries(self) -> List[STBoundary]:
        """
        Get the st boundaries

        :returns: The st boundaries
        :rtype: List[STBoundary]
        """

        return self._st_boundaries

    @property
    def min_s_on_st_boundaries(self) -> float:
        """
        Get the minimum s on the st boundaries

        :returns: The minimum s on the st boundaries
        :rtype: float
        """
        
        return self._min_s_on_st_boundaries

    @property
    def init_point(self) -> TrajectoryPoint:
        """
        Get the initial point

        :returns: The initial point
        :rtype: TrajectoryPoint
        """

        return self._init_point

    @property
    def speed_limit(self) -> SpeedLimit:
        """
        Get the speed limit

        :returns: The speed limit
        :rtype: SpeedLimit
        """

        return self._speed_limit

    @property
    def cruise_speed(self) -> float:
        """
        Get the cruise speed

        :returns: The cruise speed
        :rtype: float
        """
        
        return self._cruise_speed if self._cruise_speed > 0.0 else FLAGS_default_cruise_speed

    @property
    def path_length(self) -> float:
        """
        Get the path length

        :returns: The path length
        :rtype: float
        """

        return self._path_data_length

    @property
    def total_time_by_conf(self) -> float:
        """
        Get the total time by configuration

        :returns: The total time by configuration
        :rtype: float
        """

        return self._total_time_by_conf

    def SetSTDrivableBoundary(self, s_boundary: List[Tuple[float, float, float]],
                              v_obs_info: List[Tuple[float, float, float]]) -> bool:
        """
        Set the st drivable boundary

        :param List[Tuple[float, float, float]] s_boundary: The s boundary
        :param List[Tuple[float, float, float]] v_obs_info: The observed information
        :returns: True if the st drivable boundary is set, False otherwise
        :rtype: bool
        """

        if len(s_boundary) != len(v_obs_info):
            return False
        for i in range(len(s_boundary)):
            
            v_obs_lower = v_obs_info[i][1] if v_obs_info[i][1] > -kObsSpeedIgnoreThreshold else None
            v_obs_upper = v_obs_info[i][2] if v_obs_info[i][2] < kObsSpeedIgnoreThreshold else None
            st_bound_instance = STDrivableBoundaryInstance(s_boundary[i][0],
                                                           s_boundary[i][1],
                                                           s_boundary[i][2],
                                                           v_obs_lower,
                                                           v_obs_upper)

        self._st_drivable_boundary.st_boundary.append(st_bound_instance)
        return True

    @property
    def st_drivable_boundary(self) -> STDrivableBoundary:
        """
        Get the st drivable boundary

        :returns: The st drivable boundary
        :rtype: STDrivableBoundary
        """

        return self._st_drivable_boundary

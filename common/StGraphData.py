from protoclass.TrajectoryPoint import TrajectoryPoint
from common.SpeedLimit import SpeedLimit
from common.STBoundary import STBoundary
from typing import List,Tuple
from protoclass.STDrivableBoundary import STDrivableBoundary

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

        raise NotImplementedError

    def is_initialized(self) -> bool:
        """
        Check if the data is initialized

        :returns: True if the data is initialized, False otherwise
        :rtype: bool
        """

        return self._init

    def st_boundaries(self) -> List[STBoundary]:
        """

        """

        raise NotImplementedError

    def min_s_on_st_boundaries(self) -> float:
        """

        """
        
        raise NotImplementedError

    def init_point(self) -> TrajectoryPoint:
        """

        """

        raise NotImplementedError

    def speed_limit(self) -> SpeedLimit:
        """

        """

        raise NotImplementedError

    def cruise_speed(self) -> float:
        """

        """
        
        raise NotImplementedError

    def path_length(self) -> float:
        """
        
        """

        raise NotImplementedError

    def total_time_by_conf(self) -> float:
        """
        
        """

        raise NotImplementedError

    def SetSTDrivableBoundary(self, s_boundary: List[Tuple[float, float, float]],
                              v_obs_info: List[Tuple[float, float, float]]) -> bool:
        """
        
        """
        
        raise NotImplementedError

    def st_drivable_boundary(self) -> STDrivableBoundary:
        """
        
        """

        raise NotImplementedError

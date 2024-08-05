from dataclasses import dataclass, field
from enum import Enum
from typing import List, Tuple

class BoundType(Enum):

    ROAD = 0
    LANE = 1
    OBSTACLE = 2
    ADC = 3

@dataclass
class BoundEdge:

    type: BoundType = field(default_factory=BoundType)
    l: float = 0.0
    id: str = ''

@dataclass
class InterPolatedPoint:

    left_weight: float = None
    right_weight: float = None
    lower_bound: float = None
    left_index: int = None
    right_index: int = None
    rear_axle_s: float = None

class PathBoundPoint:

    def __init__(self, l_min: float, l_max: float, s_init: float = 0.0):
        """
        Constructor

        :param float l_min: minimum l value
        :param float l_max: maximum l value
        :param float s_init: initial s value, default is 0.0
        """
        self.s: float = s_init
        self.l_lower: BoundEdge = BoundEdge(l=l_min)
        self.l_upper: BoundEdge = BoundEdge(l=l_max)

    def __lt__(self, other: 'PathBoundPoint') -> bool:

        return self.s < other.s

class PathBoundary(list):

    def __init__(self, path_bound: List[PathBoundPoint] =[], delta_s: float = 0.0):
        """
        Constructor

        :param List[PathBoundPoint] path_bound: tuple first is s, second is l_min, third is l_max
        :param float delta_s: The distance between two point in s-axis
        """

        super.__init__(path_bound)
        self._delta_s: float = delta_s
        self._label: str = "regular"
        self._start_s: float = self.start_s()
        self._extra_path_bound: List[InterPolatedPoint] = []
        self._blocking_obstacle_id: str = ""

    @property
    def delta_s(self) -> float:
        """
        Get the delta s of the path boundary

        :returns: delta s of the path boundary
        :rtype: float
        """

        return self._delta_s

    def set_delta_s(self, s: float) -> None:
        """
        Set the delta s of the path boundary

        :param float s: delta s
        """

        self._delta_s = s

    def start_s(self)-> float:
        """
        Get the start s of the path boundary

        :returns: start s of the path boundary
        :rtype: float
        """

        if len(self) == 0:
            return 0.0
        return self[0].s

    def set_label(self, label: str) -> None:
        """
        Set the label of the path boundary

        :param str label: label of the path boundary
        """

        self._label = label

    @property
    def label(self) -> str:
        """
        Get the label of the path boundary

        :returns: label of the path boundary
        :rtype: str
        """

        return self._label

    def set_blocking_obstacle_id(self, obs_id: str) -> None:
        """
        Set the blocking obstacle id

        :param str obs_id: blocking obstacle id
        """

        self._blocking_obstacle_id = obs_id

    @property
    def blocking_obstacle_id(self) -> str:
        """
        Get the blocking obstacle id

        :returns: blocking obstacle id
        :rtype: str
        """

        return self._blocking_obstacle_id

    def boundary(self) -> List[Tuple[float, float]]:
        """
        Get the boundary of the path
        """
        boundary: List[Tuple[float, float]] = []
        for path_bound in self:
            boundary.append(path_bound.l_lower.l, path_bound.l_upper.l)
        return boundary

    def __str__(self) -> str:
        """
        Get the string representation of the path boundary

        :returns: string representation of the path boundary
        :rtype: str
        """

        return f"PathBoundary: {self.label} with {len(self)} points"

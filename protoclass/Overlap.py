from dataclasses import dataclass, field
from typing import List, Optional, Union
from ADCTrajectory import Id, Polygon

@dataclass
class LaneOverlapInfo:
    """
    LaneOverlapInfo class, oriented from protobuf message
    """

    start_s: Optional[float] = None  # position (s-coordinate)
    end_s: Optional[float] = None    # position (s-coordinate)
    is_merge: Optional[bool] = None
    region_overlap_id: Optional[Id] = None

@dataclass
class SignalOverlapInfo:
    """
    SignalOverlapInfo class, oriented from protobuf message
    """

    pass

@dataclass
class StopSignOverlapInfo:
    """
    StopSignOverlapInfo class, oriented from protobuf message
    """

    pass

@dataclass
class CrosswalkOverlapInfo:
    """
    CrosswalkOverlapInfo class, oriented from protobuf message
    """

    region_overlap_id: Optional[Id] = None

@dataclass
class JunctionOverlapInfo:
    """
    JunctionOverlapInfo class, oriented from protobuf message
    """

    pass

@dataclass
class YieldOverlapInfo:
    """
    YieldOverlapInfo class, oriented from protobuf message
    """

    pass

@dataclass
class ClearAreaOverlapInfo:
    """
    ClearAreaOverlapInfo class, oriented from protobuf message
    """

    pass

@dataclass
class SpeedBumpOverlapInfo:
    """
    SpeedBumpOverlapInfo class, oriented from protobuf message
    """

    pass

@dataclass
class ParkingSpaceOverlapInfo:
    """
    ParkingSpaceOverlapInfo class, oriented from protobuf message
    """

    pass

@dataclass
class PNCJunctionOverlapInfo:
    """
    PNCJunctionOverlapInfo class, oriented from protobuf message
    """

    pass

@dataclass
class RSUOverlapInfo:
    """
    RSUOverlapInfo class, oriented from protobuf message
    """

    pass

@dataclass
class ObjectOverlapInfo:
    """
    ObjectOverlapInfo class, oriented from protobuf message

    Information about one object in the overlap.
    """

    id: Optional[Id] = None
    overlap_info: Optional[Union[LaneOverlapInfo,
                                 SignalOverlapInfo,
                                 StopSignOverlapInfo,
                                 CrosswalkOverlapInfo,
                                 JunctionOverlapInfo,
                                 YieldOverlapInfo,
                                 ClearAreaOverlapInfo,
                                 SpeedBumpOverlapInfo,
                                 ParkingSpaceOverlapInfo,
                                 PNCJunctionOverlapInfo,
                                 RSUOverlapInfo,
                                ]
                          ] = None

@dataclass
class RegionOverlapInfo:
    """
    RegionOverlapInfo class, oriented from protobuf message
    """

    id: Optional[Id] = None
    polygon: List[Polygon] = field(default_factory=list)

@dataclass
class Overlap:
    """
    Overlap class, oriented from protobuf message

    Here, the "overlap" includes any pair of objects on the map
    (e.g. lanes, junctions, and crosswalks).
    """

    id: Optional[Id] = None
    objects: List[ObjectOverlapInfo] = field(default_factory=list)
    region_overlaps: List[RegionOverlapInfo] = field(default_factory=list)

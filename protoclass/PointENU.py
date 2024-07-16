from dataclasses import dataclass, field
import math
from typing import Optional

@dataclass
class PointENU:
    """
    PointENU class, oriented from protobuf message
    A point in the map reference frame. The map defines an origin, whose
    coordinate is (0, 0, 0).
    Most modules, including localization, perception, and prediction, generate
    results based on the map reference frame.
    Currently, the map uses Universal Transverse Mercator (UTM) projection. See
    the link below for the definition of map origin.
    https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
    The z field of PointENU can be omitted. If so, it is a 2D location and we do
    not care its height.
    """

    x: Optional[float] = field(default=math.nan)  # East from the origin, in meters.
    y: Optional[float] = field(default=math.nan)  # North from the origin, in meters.
    z: Optional[float] = field(default=0.0)       # Up from the WGS-84 ellipsoid, in meters.

from dataclasses import dataclass, field
from typing import List, Optional

@dataclass
class STDrivableBoundaryInstance:
    """
    STDrivableBoundaryInstance class, oriented from protobuf message
    """

    t: Optional[float] = None
    s_lower: Optional[float] = None
    s_upper: Optional[float] = None
    v_obs_lower: Optional[float] = None
    v_obs_upper: Optional[float] = None

@dataclass
class STDrivableBoundary:
    """
    STDrivableBoundary class, oriented from protobuf message
    """

    st_boundary: List[STDrivableBoundaryInstance] = field(default_factory=list)

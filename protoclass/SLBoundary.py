from dataclasses import dataclass, field
from typing import List, Optional

@dataclass
class SLPoint:
    pass

@dataclass
class SLBoundary:
    """
    The start_s and end_s are longitudinal values.
    start_s <= end_s.
                  end_s
                    ^
                    |
              S  direction
                    |
                start_s
    The start_l and end_l are lateral values.
    start_l <= end_l. Left side of the reference line is positive,
    and right side of the reference line is negative.
    end_l  <-----L direction---- start_l
    """

    start_s: Optional[float] = None
    end_s: Optional[float] = None
    start_l: Optional[float] = None
    end_l: Optional[float] = None
    boundary_points: List[SLPoint] = field(default_factory=list)

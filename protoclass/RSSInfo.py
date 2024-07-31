from dataclasses import dataclass
from typing import Optional

@dataclass
class RSSInfo:
    """
    RSSInfo class, oriented from protobuf message
    """

    is_rss_safe: Optional[bool] = None
    cur_dist_lon: Optional[float] = None
    rss_safe_dist_lon: Optional[float] = None
    acc_lon_range_minimum: Optional[float] = None
    acc_lon_range_maximum: Optional[float] = None
    acc_lat_left_range_minimum: Optional[float] = None
    acc_lat_left_range_maximum: Optional[float] = None
    acc_lat_right_range_minimum: Optional[float] = None
    acc_lat_right_range_maximum: Optional[float] = None

"""Apollo HD map loader brought over from ApolloAppleSilicon.

Map binaries live in ``apollo_map/data/<name>/``. They are decoded with the
bundled ``common_msgs_v9.desc`` descriptor set.
"""

from pathlib import Path

from .map.map_service import MapService
from .map.types import LaneMarking, LaneSignalOverlap, LaneStopLine, PositionEstimate
from .to_commonroad import convert_map

PACKAGE_ROOT = Path(__file__).resolve().parent
DEFAULT_DESCRIPTOR = PACKAGE_ROOT / "message" / "common_msgs_v9.desc"
MAP_DATA_DIR = PACKAGE_ROOT / "data"

__all__ = [
    "DEFAULT_DESCRIPTOR",
    "MAP_DATA_DIR",
    "LaneMarking",
    "LaneSignalOverlap",
    "LaneStopLine",
    "MapService",
    "convert_map",
    "PositionEstimate",
    "load_map",
    "map_file",
]


def map_file(map_name: str, filename: str = "base_map.bin") -> Path:
    """Return a bundled map path, such as ``borregas_ave/base_map.bin``."""
    return MAP_DATA_DIR / map_name / filename


def load_map(map_name_or_path, *, filename: str = "base_map.bin", load_only: bool = False) -> MapService:
    """Load a bundled map name or an explicit binary map path."""
    path = Path(map_name_or_path)
    if not path.is_file():
        path = map_file(str(map_name_or_path), filename)
    return MapService(DEFAULT_DESCRIPTOR, map_path=path, load_only=load_only)

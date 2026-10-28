"""Locate CommonRoad scenario files kept outside the git repository."""

from __future__ import annotations

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATASET_DIR = PROJECT_ROOT / "data" / "commonroad-scenarios"
DATASET_REPO = "https://gitlab.lrz.de/tum-cps/commonroad-scenarios.git"
DATASET_BRANCH = "2020a_scenarios"

# Highway recording used as the closed-loop road. It is part of the public
# 2020a scenario repository, fetched into DATASET_DIR.
ROAD_SCENARIO_ID = "DEU_A9-2_1_T-1"
ROAD_SCENARIO_RELATIVE = Path("scenarios/recorded/hand-crafted") / f"{ROAD_SCENARIO_ID}.xml"


def scenario_file(scenario_id: str | None = None) -> Path:
    """Return one XML inside the fetched dataset.

    ``scenario_id`` defaults to the road used by the closed-loop cases.
    """
    relative = ROAD_SCENARIO_RELATIVE if scenario_id in (None, ROAD_SCENARIO_ID) else None
    if relative is not None:
        path = DATASET_DIR / relative
        if path.is_file():
            return path
    name = f"{scenario_id or ROAD_SCENARIO_ID}.xml"
    matches = [
        path
        for path in DATASET_DIR.rglob(name)
        if path.is_file() and ".git" not in path.parts
    ]
    if matches:
        return matches[0]
    raise FileNotFoundError(
        f"CommonRoad scenario {name} is not in {DATASET_DIR}. "
        "Run python scripts/fetch_commonroad_scenarios.py"
    )

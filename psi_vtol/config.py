"""Read the vehicle, the PX4 parameters and one mission into the dictionary the simulation uses.

Each kind of number has one file:

    vehicle/<name>.yaml                 the airframe
    px4/params.yaml                     PX4 parameters
    missions/<mission>/mission.yaml     the flight profile, and which vehicle flies it
    missions/<mission>/study.yaml       what the simulation study varies

Where PX4 is built and where run files go depend on the machine, so they come from the
environment: PSI_PX4_ROOT and PSI_RUN_ROOT.
"""
import os
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
DEFAULT_MISSION = "2026_year_end"


def _read(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def profile(mission=DEFAULT_MISSION):
    """The flight profile of a mission: all the ROS 2 nodes need from these files."""
    return _read(REPO / "missions" / mission / "mission.yaml")["profile"]


def load(mission=DEFAULT_MISSION):
    """Everything the simulation needs. mission: name of a folder under missions/."""
    folder = REPO / "missions" / mission
    flight = _read(folder / "mission.yaml")
    study = _read(folder / "study.yaml")
    px4 = _read(REPO / "px4" / "params.yaml")
    vehicle = _read(REPO / "vehicle" / f"{flight['vehicle']}.yaml")
    return {
        # every value of one simulation case; any of them can be overridden or swept
        "nominal": {**vehicle, **study["environment"], **flight["profile"]},
        "sweep": study["sweep"],
        "px4_params": {**px4["concept"], **px4["per_vehicle"], **px4["sim_only"]},
        "sim": {
            # a tree with build/px4_sitl_default already built
            "px4_root": os.environ.get("PSI_PX4_ROOT", "~/px4-v1.17.0"),
            "run_root": os.environ.get("PSI_RUN_ROOT", "/tmp/psi_qp_runs"),
            **study["sim"],
        },
    }

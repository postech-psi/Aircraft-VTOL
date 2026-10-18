"""Export a reviewable QGC parameter candidate; assumed values still need measurement.

    python3 -m psi_vtol.px4_setup --output /tmp/psi_candidate.params

Never exports sim_only. It includes the failsafes of px4/params.yaml and the geofence and return
height of the mission's route. Output functions and sensor calibration are board-specific and
must be configured in QGroundControl as described in docs/03_px4_setup.md.
"""
import argparse
from pathlib import Path

from psi_vtol import config
from psi_vtol.sim import gz_model


def candidate(mission=config.DEFAULT_MISSION):
    cfg = config.load(mission)
    groups = config._read(config.REPO / "px4/params.yaml")
    cfg["px4_params"] = {**groups["concept"], **groups["per_vehicle"]}
    params = gz_model.px4_params(cfg)
    lines = ["# PSI candidate for PX4 v1.17.0; includes UNMEASURED vehicle assumptions.",
             "# Review motor geometry, thrust mapping, gains and all board-specific settings.",
             "# Includes failsafes and the mission's geofence; no sim_only, no output pin assignments.",
             "# SYSID\tCOMPID\tNAME\tVALUE\tMAV_PARAM_TYPE"]
    for name, value in sorted(params.items()):
        lines.append(f"1\t1\t{name}\t{value}\t{6 if isinstance(value, int) else 9}")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--mission", default=config.DEFAULT_MISSION)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    text = candidate(args.mission)
    if args.output:
        args.output.write_text(text)
        print(f"Wrote assumed parameter candidate: {args.output}")
    else:
        print(text, end="")


if __name__ == "__main__":
    main()

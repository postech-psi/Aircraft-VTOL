#!/usr/bin/env python3
"""Start PX4 SITL with the vehicle and keep it running, for the ROS 2 nodes to fly.

    python3 -m psi_vtol.sim.start                 # then, in two more terminals:
    MicroXRCEAgent udp4 -p 8888
    ros2 launch vtol_mission mission.launch.py auto_arm:=true

Ctrl-C stops PX4 and Gazebo. The PX4 log stays under the run folder that is printed.
"""
import argparse
import json
import signal
import uuid
from pathlib import Path

from psi_vtol import config
from psi_vtol.sim import gz_model, run_case


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--mission", default=config.DEFAULT_MISSION,
                        help="folder under missions/")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                        help="override one vehicle value; may be repeated")
    parser.add_argument("--case", default="ros2")
    parser.add_argument("--instance", type=int, default=0,
                        help="PX4 instance number; 0 gives ROS 2 topics without a namespace")
    args = parser.parse_args()

    cfg = config.load(args.mission)
    cfg["sim"]["speed_factor"] = 1      # the ROS 2 nodes run on the wall clock
    try:
        p = run_case.case_values(cfg, config.assignments(args.set))
    except (KeyError, ValueError) as error:
        parser.error(str(error))
    plugin = gz_model.config_plugin_path(cfg) / "libpsi-aerodynamics.so"
    if not plugin.is_file():
        raise SystemExit(f"Build the Gazebo plugin first: {plugin}; see docs/03_simulation.md")
    case_dir = Path(cfg["sim"]["run_root"]) / args.case / ("run-" + uuid.uuid4().hex[:12])
    case_dir.mkdir(parents=True, exist_ok=False)
    (case_dir / "manifest.json").write_text(json.dumps(run_case.provenance(cfg, p), indent=2))

    # Ctrl-C or kill, also when started in the background: stop PX4 and Gazebo on the way out
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, signal.default_int_handler)
    proc = run_case.launch(case_dir, gz_model.write_case(case_dir, p, cfg), cfg, args.instance)
    try:
        link = run_case.Link(args.instance)
        if link.conn.wait_heartbeat(timeout=90) is None:
            raise SystemExit("no heartbeat from PX4")
        for name, value in gz_model.px4_params(cfg).items():
            link.set_param(name, value)
        (case_dir / "effective_params.json").write_text(json.dumps(link.parameter_snapshot(), indent=2))
        print(f"PX4 is up with the parameters set. Run files: {case_dir}", flush=True)
        while proc.poll() is None:      # keeps the ground-station heartbeat going
            link.pump(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        run_case.stop(proc, case_dir)


if __name__ == "__main__":
    main()

"""The data files hang together: every number the simulation needs is there, once."""
import pytest
import yaml

from psi_vtol import config

PX4 = yaml.safe_load((config.REPO / "px4" / "params.yaml").read_text(encoding="utf-8"))


def test_every_mission_loads():
    for folder in (config.REPO / "missions").iterdir():
        cfg = config.load(folder.name)
        assert cfg["nominal"]["mass_kg"] > 0
        assert cfg["sim"]["autostart"] == 4004


def test_each_number_has_one_home():
    """The airframe, the air and the profile must not define the same name twice."""
    mission = config.REPO / "missions" / config.DEFAULT_MISSION
    flight = yaml.safe_load((mission / "mission.yaml").read_text(encoding="utf-8"))
    study = yaml.safe_load((mission / "study.yaml").read_text(encoding="utf-8"))
    vehicle = yaml.safe_load(
        (config.REPO / "vehicle" / f"{flight['vehicle']}.yaml").read_text(encoding="utf-8"))
    groups = [set(vehicle), set(study["environment"]), set(flight["profile"])]
    assert sum(len(g) for g in groups) == len(set().union(*groups))


def test_swept_values_exist_and_bracket_the_nominal():
    cfg = config.load()
    for name, (low, high) in cfg["sweep"].items():
        assert low < high, name
        assert low <= cfg["nominal"][name] <= high, name


def test_px4_parameter_groups_do_not_overlap():
    groups = [set(PX4["concept"]), set(PX4["per_vehicle"]), set(PX4["sim_only"])]
    assert sum(len(g) for g in groups) == len(set().union(*groups))


def test_failsafe_switches_are_sim_only():
    """Parameters that switch off RC and data-link failsafes must never reach the Pixhawk."""
    for name in ("COM_RC_IN_MODE", "COM_RCL_EXCEPT", "NAV_DLL_ACT", "NAV_RCL_ACT"):
        assert name in PX4["sim_only"]


def test_profile_is_what_the_ros_nodes_read():
    assert config.profile() == {k: config.load()["nominal"][k] for k in config.profile()}


def test_command_line_values_are_numbers_or_json_lists():
    assert config.assignments(["wind_speed_mps=4", "route_m=[[50, 0], [0, 0]]"]) == {
        "wind_speed_mps": 4.0, "route_m": [[50, 0], [0, 0]]}
    for bad in ("wind_speed_mps", "=4", "wind_speed_mps=fast", "route_m=[[50, 0]"):
        with pytest.raises(ValueError):
            config.assignments([bad])

"""The parts of the simulation that need no PX4: the Gazebo model, the case lists, the verdict."""
import xml.etree.ElementTree as ET

import pytest

from psi_vtol import config
from psi_vtol.sim import gz_model, metrics, sweep

CFG = config.load()
NOMINAL = CFG["nominal"]


def test_derived_values_match_the_airframe_data():
    d = gz_model.derived(NOMINAL)
    assert d["lift_thrust_to_weight"] == pytest.approx(1.9, abs=0.01)      # eCalc
    assert d["wing_lift_fraction_cruise"] == pytest.approx(0.12, abs=0.01)  # report.md


def test_model_has_five_motors_and_four_fixed_surfaces():
    model = ET.fromstring(gz_model.model_sdf(NOMINAL)).find("model")
    plugins = [p.get("name") for p in model.findall("plugin")]
    assert plugins.count("gz::sim::systems::MulticopterMotorModel") == 5
    assert plugins.count("gz::sim::systems::LiftDrag") == 4     # two wing halves, plate, side
    assert float(model.find("link/inertial/mass").text) == NOMINAL["mass_kg"]


def test_px4_is_told_the_nominal_geometry_in_its_own_axes():
    """PX4: x forward, y right. Motor 0 of a quad-X is front right."""
    params = gz_model.px4_params(CFG)
    assert (params["CA_ROTOR0_PX"], params["CA_ROTOR0_PY"]) == (0.3325, 0.2525)
    assert (params["CA_ROTOR1_PX"], params["CA_ROTOR1_PY"]) == (-0.3325, -0.2525)
    assert params["VT_FWD_THRUST_EN"] == 4 and params["VT_PITCH_MIN"] == 0.0


def test_wind_is_written_into_the_world():
    stock = "<world><real_time_factor>1.0</real_time_factor>\n    <spherical_coordinates/>"
    stock = stock.replace("<spherical_coordinates/>", "<spherical_coordinates>")
    calm = gz_model.world_sdf(NOMINAL, stock, 4)
    assert "<wind>" not in calm and "<real_time_factor>4</real_time_factor>" in calm
    headwind = gz_model.world_sdf(dict(NOMINAL, wind_speed_mps=6.0), stock, 4)
    assert "<linear_velocity>-6.0000 -0.0000 0</linear_velocity>" in headwind


def test_studies_have_the_cases_of_the_report():
    assert len(sweep.oat_cases(CFG)) == 48
    assert len(sweep.wind_cases()) == 40
    random = sweep.random_cases(CFG, 40, seed=1)
    assert random == sweep.random_cases(CFG, 40, seed=1)        # a seed names the same vehicles
    assert all(set(overrides) == set(CFG["sweep"]) for _, overrides in random)


def test_verdict_needs_a_complete_flight_inside_every_limit():
    good = {name: 0.0 for name in metrics.LIMITS}
    assert metrics.verdict({"completed": True}, good) == (True, [])
    assert not metrics.verdict({"completed": False, "fail_reason": "tumbled in cruise"}, good)[0]
    assert not metrics.verdict({"completed": True}, dict(good, tilt_max_deg=31.0))[0]
    assert not metrics.verdict({"completed": True}, {})[0]      # a missing figure is not a pass

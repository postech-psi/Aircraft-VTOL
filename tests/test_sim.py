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


def test_model_has_five_motors_and_all_angle_aero_without_double_counting_drag():
    model = ET.fromstring(gz_model.model_sdf(NOMINAL)).find("model")
    plugins = [p.get("name") for p in model.findall("plugin")]
    assert plugins.count("gz::sim::systems::MulticopterMotorModel") == 5
    assert plugins.count("psi::AeroSystem") == 1
    aero = model.find("plugin[@name='psi::AeroSystem']")
    assert len(aero.findall("surface")) == 3  # two wing halves and the rear plate
    assert len(aero.findall("rotor")) == 5
    assert float(aero.find("surface/cm0").text) == NOMINAL["wing_cm0"]
    for motor in model.findall("plugin[@name='gz::sim::systems::MulticopterMotorModel']"):
        assert float(motor.find("rotorDragCoefficient").text) == 0
    assert float(model.find("link/inertial/mass").text) == NOMINAL["mass_kg"]


def test_px4_is_told_the_nominal_geometry_in_its_own_axes():
    """PX4: x forward, y right. Motor 0 of a quad-X is front right."""
    params = gz_model.px4_params(CFG)
    assert (params["CA_ROTOR0_PX"], params["CA_ROTOR0_PY"]) == (0.3325, 0.2525)
    assert (params["CA_ROTOR1_PX"], params["CA_ROTOR1_PY"]) == (-0.3325, -0.2525)
    assert params["VT_FWD_THRUST_EN"] == 5 and params["VT_PITCH_MIN"] == 0.0
    assert params["CA_ROTOR2_KM"] == -NOMINAL["lift_moment_m"]


def test_wind_is_written_into_the_world():
    stock = "<world><real_time_factor>1.0</real_time_factor>\n    <spherical_coordinates/>"
    stock = stock.replace("<spherical_coordinates/>", "<spherical_coordinates>")
    calm = gz_model.world_sdf(NOMINAL, stock, 4)
    assert "<wind>" not in calm and "<real_time_factor>4</real_time_factor>" in calm
    headwind = gz_model.world_sdf(dict(NOMINAL, wind_speed_mps=6.0), stock, 4)
    assert "<linear_velocity>-6.0000 -0.0000 0.0000</linear_velocity>" in headwind
    vertical = gz_model.world_sdf(dict(NOMINAL, wind_vertical_mps=2), stock, 1)
    assert "2.0000</linear_velocity>" in vertical


def test_studies_cover_the_new_uncertainties():
    assert len(sweep.oat_cases(CFG)) > 48  # original report was for the legacy model
    assert any("wing_cm0" in p for _, p in sweep.oat_cases(CFG))
    assert len(sweep.wind_cases()) == 64
    assert len(sweep.corner_cases()) == 7
    random = sweep.random_cases(CFG, 40, seed=1)
    assert random == sweep.random_cases(CFG, 40, seed=1)        # a seed names the same vehicles
    assert all(set(overrides) == set(CFG["sweep"]) for _, overrides in random)


def test_verdict_needs_a_complete_flight_inside_every_limit():
    good = dict({name: 0.0 for name in metrics.LIMITS}, landed_confirmed=True, aero_covers_flight=True)
    assert metrics.verdict({"completed": True}, good) == (True, [])
    assert not metrics.verdict({"completed": False, "fail_reason": "tumbled in cruise"}, good)[0]
    assert not metrics.verdict({"completed": True}, dict(good, tilt_max_deg=31.0))[0]
    assert not metrics.verdict({"completed": True}, {})[0]      # a missing figure is not a pass


@pytest.mark.parametrize("name", list(metrics.LIMITS))
def test_nonfinite_metrics_never_pass(name):
    good = dict({n: 0.0 for n in metrics.LIMITS}, landed_confirmed=True, aero_covers_flight=True)
    good[name] = float("nan")
    assert not metrics.verdict({"completed": True}, good)[0]


def test_tracking_landing_and_lower_saturation_are_required():
    good = dict({n: 0.0 for n in metrics.LIMITS}, landed_confirmed=True, aero_covers_flight=True)
    for field in ("cruise_speed_err_mps", "touchdown_err_m", "lift_low_frac", "fwd_yaw_err_max_deg"):
        assert not metrics.verdict({"completed": True}, dict(good, **{field: 100}))[0]
    assert not metrics.verdict({"completed": True}, dict(good, landed_confirmed=False))[0]


def test_nominal_yaw_coefficients_survive_px4_weak_row_cutoff():
    params = gz_model.px4_params(config.load())
    coefficients = [params[f"CA_ROTOR{i}_CT"] * params[f"CA_ROTOR{i}_KM"] for i in range(4)]
    # PX4 v1.17 ControlAllocator.cpp removes a row when every entry is <= 0.05.
    assert max(abs(v) for v in coefficients) > .05
    assert all(params[f"CA_ROTOR{i}_CT"] == params["CA_ROTOR4_CT"] for i in range(4))
    assert coefficients[0] == -coefficients[2]

"""The parts of the simulation that need no PX4: the Gazebo model, the case lists, the verdict."""
import xml.etree.ElementTree as ET

import pytest

from psi_vtol import config
from psi_vtol.sim import gz_model, metrics, run_case, sweep

CFG = config.load()
NOMINAL = CFG["nominal"]


def test_derived_values_match_the_airframe_data():
    d = gz_model.derived(dict(NOMINAL, cruise_mps=10.0))
    assert d["lift_thrust_to_weight"] == pytest.approx(1.9, abs=0.01)      # eCalc
    assert d["wing_lift_fraction_cruise"] == pytest.approx(0.12, abs=0.01)  # 10 m/s, 2026-10-08 report


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
    assert params["VT_FWD_THRUST_EN"] == 6 and params["VT_PITCH_MIN"] == 0.0
    assert params["CA_ROTOR2_KM"] == -NOMINAL["lift_moment_m"]


def test_a_front_transition_can_never_finish():
    """No control surfaces: a transition must be cancelled before it reaches fixed-wing flight."""
    params = gz_model.px4_params(CFG)
    assert params["VT_F_TRANS_THR"] == 0.0                     # the pusher stays off
    assert params["VT_ARSP_TRANS"] >= 30.0                     # twice the pusher's airspeed limit
    assert params["VT_TRANS_TIMEOUT"] < params["VT_F_TRANS_DUR"]   # no airspeed: timeout first


def test_failsafes_follow_the_route():
    params = gz_model.px4_params(CFG)
    assert (params["COM_OBL_RC_ACT"], params["GF_ACTION"], params["COM_LOW_BAT_ACT"]) == (5, 3, 3)
    assert params["RTL_TYPE"] == 0 and params["RTL_RETURN_ALT"] == NOMINAL["alt_m"]
    assert params["GF_MAX_HOR_DIST"] == 101.0                  # 50 m square: corner at 70.7 m
    long_route = gz_model.px4_params(CFG, dict(NOMINAL, route_m=[[300.0, 0.0], [0.0, 0.0]]))
    assert long_route["GF_MAX_HOR_DIST"] == 330.0
    overridden = dict(CFG, px4_params=dict(CFG["px4_params"], GF_MAX_HOR_DIST=500.0))
    assert gz_model.px4_params(overridden)["GF_MAX_HOR_DIST"] == 500.0   # --px4 still wins


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


def test_scenarios_fly_every_speed_in_both_modes_and_each_event():
    cases = dict(sweep.scenario_cases())
    for through in (0, 1):
        speeds = {c["cruise_mps"] for c in cases.values()
                  if c.get("fly_through") == through and "event" not in c and "wind_speed_mps" not in c}
        assert speeds == {3.0, 4.0, 5.0, 6.0, 7.0}
    events = {c["event"].partition("@")[0] for c in cases.values() if "event" in c}
    assert events == set(sweep.EXPECTED) == set(run_case.EVENTS)
    for c in cases.values():
        if "event" in c:
            run_case.parse_event(c["event"])


def test_an_event_case_is_judged_by_the_response_it_tests():
    def record(event, **flight):
        return {"event": event, "flight": {"event": {"name": event.partition("@")[0]}, **flight}}
    assert sweep.expected_response({"event": None, "flight": {}}) == (True, "")
    assert sweep.expected_response(record("abort@cruise2+1",
        failsafe={"handed_to": "Hold"}, px4_modes=["Offboard", "Hold", "Land"]))[0]
    assert not sweep.expected_response(record("abort@cruise2+1", failsafe={"handed_to": "Land"}))[0]
    assert sweep.expected_response(record("silence@cruise2+1", px4_modes=["Hold", "Land"]))[0]
    assert not sweep.expected_response(record("silence@cruise2+1", px4_modes=["Position"]))[0]
    pusher = record("pusher_off@cruise2+1", failsafe={"reason": "pusher: losing speed", "handed_to": "Land"})
    assert sweep.expected_response(pusher)[0]
    assert not sweep.expected_response(record("pusher_off@cruise2+1",
        failsafe={"reason": "falling behind the route", "handed_to": "Hold"}))[0]
    assert sweep.expected_response(record("transition@hover1+2", completed=True))[0]
    assert sweep.expected_response(record("battery@cruise2+1", px4_modes=["Offboard", "Return"],
                                          after={"from_home_m": 0.4}))[0]
    assert not sweep.expected_response(record("battery@cruise2+1", px4_modes=["Land"],
                                              after={"from_home_m": 40.0}))[0]
    assert not sweep.expected_response({"event": "abort@cruise9+1", "flight": {}})[0]   # never fired


def test_a_failsafe_is_safe_when_it_ends_on_the_ground_unharmed():
    down = {"landed_confirmed": True, "tilt_max_deg": 12.0, "touchdown_speed_mps": 0.5}
    assert metrics.outcome({"fail_reason": "mission aborted"}, down) == (True, [])
    assert not metrics.outcome({"fail_reason": "mission aborted; tumbled"}, down)[0]
    assert not metrics.outcome({"fail_reason": None}, dict(down, landed_confirmed=False))[0]
    assert not metrics.outcome({"fail_reason": None}, dict(down, touchdown_speed_mps=1.5))[0]


def test_verdict_needs_a_complete_flight_inside_every_limit():
    good = dict({name: 0.0 for name in metrics.LIMITS}, landed_confirmed=True, aero_covers_flight=True)
    assert metrics.verdict({"completed": True}, good) == (True, [])
    assert not metrics.verdict({"completed": False, "fail_reason": "tumbled in cruise"}, good)[0]
    assert not metrics.verdict({"completed": True}, dict(good, tilt_max_deg=31.0))[0]
    assert not metrics.verdict({"completed": True}, {})[0]      # a missing figure is not a pass
    no_cruise = {"completed": True, "phases": {"accel1": [0, 5], "decel1": [5, 10]}}
    assert metrics.verdict(no_cruise, {k: v for k, v in good.items() if k != "cruise_speed_err_mps"})[0]
    cruise = {"completed": True, "phases": {"accel1": [0, 5], "cruise1": [5, 8]}}
    assert not metrics.verdict(cruise, {k: v for k, v in good.items() if k != "cruise_speed_err_mps"})[0]


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

"""What Gazebo and PX4 are told: the vehicle model, the world and PX4's parameters."""
import xml.etree.ElementTree as ET

import pytest

from psi_vtol import config, px4_setup
from psi_vtol.sim import gz_model

CFG = config.load()
NOMINAL = CFG["nominal"]


def test_derived_values_match_the_airframe_data():
    d = gz_model.derived(dict(NOMINAL, cruise_mps=10.0))
    assert d["lift_thrust_to_weight"] == pytest.approx(1.9, abs=0.01)      # eCalc
    assert d["wing_lift_fraction_cruise"] == pytest.approx(0.12, abs=0.01)  # at 10 m/s


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
    stock = "<world><real_time_factor>1.0</real_time_factor>\n    <spherical_coordinates>"
    calm = gz_model.world_sdf(NOMINAL, stock, 4)
    assert "<wind>" not in calm and "<real_time_factor>4</real_time_factor>" in calm
    headwind = gz_model.world_sdf(dict(NOMINAL, wind_speed_mps=6.0), stock, 4)
    assert "<linear_velocity>-6.0000 -0.0000 0.0000</linear_velocity>" in headwind
    vertical = gz_model.world_sdf(dict(NOMINAL, wind_vertical_mps=2), stock, 1)
    assert "2.0000</linear_velocity>" in vertical


def test_nominal_yaw_coefficients_survive_px4_weak_row_cutoff():
    params = gz_model.px4_params(CFG)
    coefficients = [params[f"CA_ROTOR{i}_CT"] * params[f"CA_ROTOR{i}_KM"] for i in range(4)]
    # PX4 v1.17 ControlAllocator.cpp removes a row when every entry is <= 0.05.
    assert max(abs(v) for v in coefficients) > .05
    assert all(params[f"CA_ROTOR{i}_CT"] == params["CA_ROTOR4_CT"] for i in range(4))
    assert coefficients[0] == -coefficients[2]


def test_the_pixhawk_candidate_leaves_out_the_simulation_switches():
    text = px4_setup.candidate()
    assert "CA_ROTOR4_AX\t1.0" in text                       # QGC format: name, value, type
    for name in config._read(config.REPO / "px4/params.yaml")["sim_only"]:
        assert f"\t{name}\t" not in text

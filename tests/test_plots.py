"""plots.py: the small pieces of logic behind the figures."""
from psi_vtol import plots


def test_phases_group_into_take_off_legs_and_landing():
    phases = {"climb": [0, 10], "hover1": [10, 16], "accel1": [16, 21], "cruise1": [21, 26],
              "decel1": [26, 31], "hover2": [31, 37], "turn2": [37, 43], "accel2": [43, 48],
              "cruise2": [48, 53], "decel2": [53, 58], "hover3": [58, 64], "descend": [64, 90],
              "land": [90, 95]}
    assert plots.flight_parts(phases) == [[0, 16, "Take-off"], [16, 37, "Leg 1"],
                                          [37, 64, "Leg 2"], [64, 95, "Landing"]]


def test_case_labels_put_units_the_usual_way():
    label = lambda **o: plots.case_label({"overrides": o})
    assert label() == "Nominal vehicle"
    assert label(mass_kg=6.0) == "Mass 6 kg"
    assert label(wing_incidence_deg=4.0) == "Wing incidence 4°"
    assert label(wind_speed_mps=8.0, wind_from_deg=90.0) == "Wind 8 m/s from 90°"
    assert label(wind_speed_mps=5.0, wind_gust_pct=0.4) == "Gusts 40% of the mean (5 m/s wind)"


def test_wind_cases_are_read_from_their_names():
    records = [{"case": "wind8_from90", "passed": True, "reasons": []},
               {"case": "nominal", "passed": True, "reasons": []}]
    assert {k: r["passed"] for k, r in plots.wind_cases(records).items()} == {(8.0, 90.0): True}


def test_failure_reasons_are_sorted_by_severity():
    assert plots.reason({"reasons": ["tilt_max_deg 39.3 > 30.0", "pusher_sat_frac 0.2 > 0.1"]}) == "tilt"
    assert plots.reason({"reasons": ["timeout in accel"]}) == "pusher"
    assert plots.reason({"reasons": ["fwd_yaw_err_max_deg 21.9 > 20.0"]}) == "heading"
    assert plots.reason({"reasons": ["touchdown_speed_mps 1.05 > 1.0"]}) == "other"

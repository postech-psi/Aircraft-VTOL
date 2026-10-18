"""sweep.py: the studies, how an event case is judged, and how a case reaches the runner."""
import queue
from types import SimpleNamespace

import pytest

from psi_vtol import config
from psi_vtol.sim import run_case, sweep

CFG = config.load()


def test_the_studies_have_the_cases_the_docs_list():
    assert len(sweep.oat_cases(CFG)) == 61
    assert len(sweep.wind_cases()) == 64
    assert len(sweep.corner_cases()) == 7
    assert len(sweep.scenario_cases()) == 30
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
    pusher = record("pusher_off@cruise2+1", failsafe={"reason": "pusher: not speeding up", "handed_to": "Land"})
    assert sweep.expected_response(pusher)[0]
    assert not sweep.expected_response(record("pusher_off@cruise2+1",
        failsafe={"reason": "falling behind the route", "handed_to": "Land"}))[0]
    assert sweep.expected_response(record("transition@hover1+2", completed=True))[0]
    assert sweep.expected_response(record("battery@cruise2+1", px4_modes=["Offboard", "Return"],
                                          after={"from_home_m": 0.4}))[0]
    assert not sweep.expected_response(record("battery@cruise2+1", px4_modes=["Land"],
                                              after={"from_home_m": 40.0}))[0]
    assert not sweep.expected_response({"event": "abort@cruise9+1", "flight": {}})[0]   # never fired
    assert sweep.case_ok({"event": "abort@cruise2+1", "safe": True, "response_met": True})
    assert not sweep.case_ok({"event": "abort@cruise2+1", "safe": False, "response_met": True})


@pytest.fixture
def runner(monkeypatch, tmp_path):
    """run_one against a runner that fails at once. Returns the commands it was given."""
    monkeypatch.setattr(config, "REPO", tmp_path)
    commands = []
    def run(cmd, **kwargs):
        commands.append(cmd)
        return SimpleNamespace(returncode=1, stderr="failed")
    monkeypatch.setattr(sweep.subprocess, "run", run)
    instances = queue.Queue()
    instances.put(5)
    def run_one(case, overrides, study):
        (tmp_path / "out/test" / sweep.MODEL_REVISION / study).mkdir(parents=True, exist_ok=True)
        return sweep.run_one(case, overrides, {}, "test", instances, study)
    run_one.commands, run_one.instances, run_one.root = commands, instances, tmp_path
    return run_one


def test_runner_failure_never_reuses_old_success(runner):
    output = runner.root / "out/test" / sweep.MODEL_REVISION / "oat/nominal.json"
    output.parent.mkdir(parents=True)
    output.write_text('{"passed": true}')
    result = runner("nominal", {}, "oat")
    assert not result["passed"] and "runner failed" in result["reasons"][0]
    assert runner.instances.get_nowait() == 5                   # the instance is given back


def test_an_event_reaches_the_runner(runner):
    runner("abort", {"event": "abort@cruise2+1.5", "cruise_mps": 7.0}, "scenarios")
    cmd = runner.commands[0]
    assert cmd[cmd.index("--event") + 1] == "abort@cruise2+1.5"
    assert cmd[cmd.index("--set") + 1] == "cruise_mps=7.0"

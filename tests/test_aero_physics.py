"""Compile and exercise the production C++ force model without Gazebo."""
import shutil
import subprocess

import pytest

from psi_vtol import config


def test_all_angle_physics_and_propulsion(tmp_path):
    compiler = shutil.which("c++")
    if compiler is None:
        pytest.skip("C++ compiler unavailable; run this check on the Gazebo host")
    binary = tmp_path / "aero_physics_test"
    subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-fsanitize=undefined", "-I", str(config.REPO / "simulation/gazebo"),
                    str(config.REPO / "tests/aero_physics_test.cc"), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)

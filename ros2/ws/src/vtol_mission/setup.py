import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vtol_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PSI VTOL team',
    maintainer_email='todo@todo.todo',
    description='Mission nodes of the PSI VTOL: step sequencing, setpoint mux, path and landing',
    license='TODO',
    entry_points={
        'console_scripts': [
            'mission_manager = vtol_mission.mission_manager:main',
            'setpoint_mux = vtol_mission.setpoint_mux:main',
            'path_node = vtol_mission.path_node:main',
            'landing_node = vtol_mission.landing_node:main',
        ],
    },
)

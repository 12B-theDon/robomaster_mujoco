from setuptools import find_packages
from setuptools import setup

setup(
    name='mujoco_rm_bringup',
    version='0.0.1',
    packages=find_packages(
        include=('mujoco_rm_bringup', 'mujoco_rm_bringup.*')),
)

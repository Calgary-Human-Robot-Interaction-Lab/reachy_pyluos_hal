#!/usr/bin/env python

from setuptools import find_packages, setup


setup(
    name='reachy_pyluos_hal',
    version='0.1.0',
    packages=find_packages(exclude=['tests']),

    install_requires=[
        'reachy_ros_hal',
    ],

    author='Pollen Robotics',
    author_email='contact@pollen-robotics.com',
    url='https://github.com/pollen-robotics/reachy_mockup_hardware',

    description='Reachy HAL implementation via Pyluos',
)
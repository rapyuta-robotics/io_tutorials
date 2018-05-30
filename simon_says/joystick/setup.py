#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(version='1.0.0', packages=['joystick'], package_dir={'': 'src'})

setup(**d)

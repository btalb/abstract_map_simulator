#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['human_cues_tag_simulator'], package_dir={'': 'src'})

setup(**d)

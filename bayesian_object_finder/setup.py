#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['bayesian_object_finder'],
    package_dir={'': 'src'},
)

setup(**setup_args)

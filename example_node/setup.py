#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
# TODO: Change the name here to your package name
setup_args = generate_distutils_setup(
     packages=['example_node'],
     package_dir={'': 'src'}
)

setup(**setup_args)
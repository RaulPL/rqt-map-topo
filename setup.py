#!/usr/bin/env python

__author__ = 'Raul Peralta Lozada'


from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_map_topo'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_map_topo']
)

setup(**d)
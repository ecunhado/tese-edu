#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['a50_dvl_algorithms', 'a50_dvl_ros'],
 package_dir={'a50_dvl_algorithms': 'src/a50_dvl_algorithms', 'a50_dvl_ros': 'src/a50_dvl_ros'}
)

setup(**d)

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['assignment1_exp_rob_lab']
d['package_dir'] = {'': 'scripts'}

setup(**d)
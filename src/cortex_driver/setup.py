from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['cortex_driver_node'],
    package_dir={'cortex_driver_node': 'cortex_driver_node'},
)

setup(**setup_args)

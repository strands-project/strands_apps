from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['watchdog_node','watchdog_node.monitors','watchdog_node.actions'],
    package_dir={'': 'src'}
)

setup(**d)

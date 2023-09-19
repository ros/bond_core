from setuptools import setup, find_packages

package_name = 'bondpy'

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['bondpy'],
    package_dir={'': 'python'},
    maintainer='Geoffrey Biggs',
    maintainer_email='geoff@openrobotics.org',
)

setup(**d)

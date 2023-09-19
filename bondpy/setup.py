from setuptools import setup, find_packages

package_name = 'bondpy'

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['bondpy'],
    package_dir={'': 'python'},
    maintainer='Geoffrey Biggs',
    maintainer_email='geoff@openrobotics.org',
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name])
    ]
)

setup(**d)

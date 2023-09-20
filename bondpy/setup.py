from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

package_name = 'bondpy'

d = generate_distutils_setup(
    packages=['bondpy'],
    package_dir={'': ''},
    maintainer='Geoffrey Biggs',
    maintainer_email='geoff@openrobotics.org',
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name])
    ],
    tests_require=['pytest']
)

setup(**d)

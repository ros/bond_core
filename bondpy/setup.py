from setuptools import setup, find_packages

package_name = 'bondpy'

setup(
    name=package_name,
    version='1.8.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Carroll',
    maintainer_email='michael@openrobotics.org',
    description='Python implementation of bond, a mechanism for checking when another process has terminated.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
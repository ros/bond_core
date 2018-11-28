#!/usr/bin/python3.5

from setuptools import find_packages
from setuptools import setup

package_name = 'smclibpy'
setup(
    name= package_name,
    version='1.8.4',
    packages=find_packages(),
    py_modules=[
        'python/smclib/statemap'],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Various',
    author_email='',
    maintainer='Michael Carroll',
    maintainer_email='michael@openrobotics.org',
    url=['http://smc.sourceforge.net','https://github.com/ros/bond_core'],
    download_url='https://github.com/ros2/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='The State Machine Compiler (SMC) converts a language-independent description of a state machine into the source code to support that state machine.',
    long_description="""\
The State Machine Compiler (SMC) from http://smc.sourceforge.net/
    converts a language-independent description of a state machine
    into the source code to support that state machine.

    This package contains the libraries that a compiled state machine
depends on, but it does not contain the compiler itself.""",
    license=['Apache License, Version 2.0', 'Mozilla Public License Version 1.1'],
    tests_require=['pytest'],
    entry_points={
        'smclibpy.python.smclib': [
            'statemap = smclibpy.python.smclib.statemap:StateUndefinedException'
        ],
        
    },
)

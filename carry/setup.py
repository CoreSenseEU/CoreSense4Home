#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import find_packages, setup
from glob import glob

NAME = "carry"


setup(
    name=NAME,
    version="0.1.0",
    license="Apache-2.0",
    description="Carry my luggage demo",
    author="todo",
    author_email="todo@todo.todo",
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + NAME, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['res/' + NAME]),
        ('share/ament_index/resource_index/pal_system_module', ['module/' + NAME]),
        ('share/' + NAME + '/launch', glob('launch/*.launch.py')),
        ('share/' + NAME + '/module', ['module/' + NAME + '_module.yaml']),
        # uncomment the next two lines to make the application be launched automatically at startup
        # ('share/ament_index/resource_index/pal_system_module_set', ['module_set/' + NAME]),
        # ('share/' + NAME + '/module_set', ['module_set/' + NAME + '_module_set.yaml'])
    ],
    tests_require=['pytest'],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'run_app = ' + NAME + '.run_app:main'
        ],
    },
)

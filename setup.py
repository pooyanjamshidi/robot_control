# -*- coding: utf-8 -*-

from setuptools import setup, find_packages

setup(
    name='robotcontrol',
    version='0.1.0',
    description='Robot controllers for BRASS MARS project',
    author='Pooyan Jamshidi',
    author_email='pooyan.jamshidi@gmail.com',
    url='https://github.com/pooyanjamshidi',
    packages=find_packages(exclude=('tests', 'docs'))
)


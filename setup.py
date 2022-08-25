#!/usr/bin/env python
import os

from setuptools import setup,find_packages

setup(
    name="unitree_api_wrapper",
    version="1.0.1",
    install_requires=["tqdm", "matplotlib", "torch"],
    extras_require={},
    packages=find_packages(exclude=("lib", "data", "policies", "scripts",)),
)

#!/usr/bin/env python
import os

from setuptools import setup

setup(
    name="unitree_api_wrapper",
    version="1.0",
    install_requires=["tqdm", "matplotlib", "torch"],
    extras_require={},
    packages=["unitree_api_wrapper"]
)

from setuptools import find_packages
from setuptools import setup

setup(
    name='axlr_controller',
    version='0.0.0',
    packages=find_packages(
        include=('axlr_controller', 'axlr_controller.*')),
)

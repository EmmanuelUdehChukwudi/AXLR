from setuptools import find_packages
from setuptools import setup

setup(
    name='axlr_localisation',
    version='0.0.0',
    packages=find_packages(
        include=('axlr_localisation', 'axlr_localisation.*')),
)

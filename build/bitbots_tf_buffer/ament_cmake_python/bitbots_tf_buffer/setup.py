from setuptools import find_packages
from setuptools import setup

setup(
    name='bitbots_tf_buffer',
    version='1.0.0',
    packages=find_packages(
        include=('bitbots_tf_buffer', 'bitbots_tf_buffer.*')),
)

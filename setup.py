#!/usr/bin/env python

from setuptools import setup, find_packages
from os import path

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()


setup(
    name='cnspy_trajectory',
    version="0.2.8",
    author='Roland Jung',
    author_email='roland.jung@aau.at',    
    description='Trajectory in SE(3) space with utilities.',
    long_description=long_description,
    long_description_content_type="text/markdown",
    url='https://github.com/aau-cns/cnspy_trajectory/',
    project_urls={
        "Bug Tracker": "https://github.com/aau-cns/cnspy_trajectory/issues",
    },    
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Operating System :: OS Independent",
    ],
    
    packages=find_packages(exclude=["test_*", "TODO*"]),
    python_requires='>=3.6',
    install_requires=['numpy', 'matplotlib', 'cnspy_csv2dataframe>=0.2.2', 'spatialmath-python', 'cnspy_spatial_csv_formats>=0.2.2'],
    entry_points={
        'console_scripts': [
            'PlotTrajectory = cnspy_trajectory.PlotTrajectory:main',
        ],
    },
)

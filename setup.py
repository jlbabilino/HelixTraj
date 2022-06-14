from setuptools import setup

setup(
    name='HelixTrajectory',
    version='0.0.0',
    description='Triple Helix Trajectory Generation Utility',
    author='Joshua Nichols',
    packages=['helixtrajectory', 'helixtrajectory.drive'],
    install_requires=[
        'casadi>=3.5.5',
        'numpy>=1.19.3'
    ],
    include_package_data=True
)
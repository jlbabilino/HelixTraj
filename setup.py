from setuptools import setup, find_packages

with open('README.md', 'r') as fh:
    long_description = fh.read()

setup(
    name='HelixTrajectory',
    version='0.0.0',
    author='Joshua Nichols',
    license='MIT',
    description='Triple Helix Trajectory Generation Utility',
    long_description=long_description,
    py_modules=['cli', 'app'],
    packages=find_packages(),
    install_requires=[
        'casadi>=3.5.5',
        'numpy>=1.19.3', # TODO: try later versions
        'click>=8.1.3'
    ],
    python_requires='>=3.8',
    classifiers=[
        'Programming Language :: Python :: 3.8',
        'Operating System :: OS Independent'
    ],
    entry_points={
        'console_scripts': [
            'helixtrajectory = cli:cli'
        ]
    }
)
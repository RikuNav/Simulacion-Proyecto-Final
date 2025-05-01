import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'mlr_nav2_puzzlebot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paolo Reyes | Ricardo Navarro | Fernando Matute | Alain Vicencio',
    maintainer_email='paolo.alfonso.reyes@gmail.com | ricardo.navgom@gmail.com | ferjosums@gmail.com | alvaofic@gmail.com',
    description='Puzzlebot navigation using nav2',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)

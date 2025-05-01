from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mlr_nav2_puzzlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
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

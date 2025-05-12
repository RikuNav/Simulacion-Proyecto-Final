import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'mlr_nav2_puzzlebot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.sdf'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'plugins'), glob(os.path.join('plugins', '*.so'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.pgm'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paolo Reyes | Ricardo Navarro | Fernando Matute | Alain Vicencio',
    maintainer_email='paolo.alfonso.reyes@gmail.com | ricardo.navgom@gmail.com | ferjosums@gmail.com | alvaofic@gmail.com',
    description='Puzzlebot navigation using nav2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'puzzlebot_localization = mlr_nav2_puzzlebot.puzzlebot_localization:main',
            'puzzlebot_joint_state_publisher = mlr_nav2_puzzlebot.puzzlebot_joint_state_publisher:main',
        ],
    },
)

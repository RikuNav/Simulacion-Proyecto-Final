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

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ]+ [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('urdf') for file in files
    ]+
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('meshes') for file in files
    ]+
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('models') for file in files
    ]+
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('worlds') for file in files
    ]
    +
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('plugins') for file in files
    ],  
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paolo Reyes | Ricardo Navarro | Fernando Matute | Alain Vicencio',
    maintainer_email='paolo.alfonso.reyes@gmail.com | ricardo.navgom@gmail.com | ferjosums@gmail.com | alvaofic@gmail.com',
    description='Puzzlebot navigation using nav2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'movement = mlr_nav2_puzzlebot.movement:main',
            'puzzlebot_gazebo = mlr_nav2_puzzlebot.puzzlebot_gazebo:main',
        ],
    },
)

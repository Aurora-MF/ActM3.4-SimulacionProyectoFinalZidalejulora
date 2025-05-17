from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'zaidalejulora_nav2_puzzlebot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*.yaml'))),
        (os.path.join('share', package_name, 'map'), glob(os.path.join('map', '*.yaml')) + glob(os.path.join('map', '*.pgm')))
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
    maintainer='Aurora Martinez | Zaida Lopez |',
    maintainer_email='A01709293@tec.mx | A01708755@tec.mx',
    description='Autonomous Puzzlebot mapping and navigation with SLAM toolbox.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localisation = zaidalejulora_nav2_puzzlebot.localisation:main',
            'joint_state_publisher = zaidalejulora_nav2_puzzlebot.joint_state_publisher:main'
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur5e_golf_pick_place'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nelsonj',
    maintainer_email='nelsonjorvany@gmail.com',
    description='UR5e Golf Ball Pick and Place Task',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pick_place_controller = ur5e_golf_pick_place.pick_place_controller:main',
            'pick_and_place = ur5e_golf_pick_place.pick_place_exercise.Pick_and_place:main',
        ],
    },
)

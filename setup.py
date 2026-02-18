import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bobble_controllers'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'foxglove'), glob('foxglove/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krallapalli',
    maintainer_email='todo@todo.com',
    description='PID balance controller for BobbleBot using EKF-filtered orientation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'balance_node = bobble_controllers.balance_node:main',
        ],
    },
)

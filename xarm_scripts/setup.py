from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'xarm_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vinman',
    maintainer_email='vinman.cub@gmail.com',
    description='Python scripts for xArm robot testing and utilities',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_pymoveit2_api = xarm_scripts.test_pymoveit2_api:main',
            'moveit_py_joint_goal = xarm_scripts.moveit_py_joint_goal:main',
            'moveit_py_pose_goal = xarm_scripts.moveit_py_pose_goal:main',
        ],
    },
)


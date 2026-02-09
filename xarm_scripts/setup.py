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
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
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
            'example_joint_goal = xarm_scripts.example_joint_goal:main',
            'example_pose_goal = xarm_scripts.example_pose_goal:main',
            'example_robot_state = xarm_scripts.example_robot_state:main',
            'example_planning_scene = xarm_scripts.example_planning_scene:main',
            'example_kinematic_constraints = xarm_scripts.example_kinematic_constraints:main',
            'example_robot_trajectory = xarm_scripts.example_robot_trajectory:main',
            'example_transforms = xarm_scripts.example_transforms:main',
            'example_advanced_planning = xarm_scripts.example_advanced_planning:main',
            'example_multi_pipeline = xarm_scripts.example_multi_pipeline:main',
            'example_collision = xarm_scripts.example_collision:main',
            'pick_and_place = xarm_scripts.pick_and_place:main',
            'pickplace_mtc_planning = xarm_scripts.pickplace_mtc_planning:main',
            'pickplace_mtc_gazebo = xarm_scripts.pickplace_mtc_gazebo:main',
            'pickplace_project_final = xarm_scripts.pickplace_project_final:main',
            'example_gemini_detection = xarm_scripts.example_gemini_detection:main',
            'color_detector = xarm_scripts.color_detector:main',
            'color_sorter = xarm_scripts.color_sorter:main',
        ],
    },
)


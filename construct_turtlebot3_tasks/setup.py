from setuptools import find_packages, setup

package_name = 'construct_turtlebot3_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arclab',
    maintainer_email='rkdtjtneh@kyonggi.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sub_obs_det = construct_turtlebot3_tasks.subscriber_obstacle_detector:main ',
            'move_rover = construct_turtlebot3_tasks.publish_bot_move:main',
            'plant_detector = construct_turtlebot3_tasks.plant_detector_node:main',
            'autonomous_drive = construct_turtlebot3_tasks.autonomous_exploration:main'

        ],
    },
)

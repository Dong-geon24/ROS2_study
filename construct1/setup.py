from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'construct1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
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
            'hello = construct1.test1:main',
            'bye = construct1.test1:main_shutdown',
            'test1_long = construct1.test1_long:main',
            'node_test1 = construct1.test_node:main',
            'node_test2 = construct1.test_node:main2',
            'temperature = construct1.temperature_monitor:main'

        ],      
    },
)

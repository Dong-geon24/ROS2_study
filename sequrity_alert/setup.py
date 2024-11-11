from setuptools import find_packages, setup

package_name = 'sequrity_alert'

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
    maintainer='rokey',
    maintainer_email='rkdtjtneh@kyonggi.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_pub = sequrity_alert.image_publisher:main',
            'img_sub = sequrity_alert.image_subscriber:main',
            'yolo_pub = sequrity_alert.yolo_publisher:main',
            'yolo_sub = sequrity_alert.yolo_sub:main',
            'zone_req = sequrity_alert.zone_request_srv:main',
            'zone_get = sequrity_alert.test_robot_req_sub:main'
        ],
    },
)

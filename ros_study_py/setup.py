from setuptools import find_packages, setup

package_name = 'ros_study_py'

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
    maintainer='donggeonyoon',
    maintainer_email='rkdtjtneh@kyonggi.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_msg_test = ros_study_py.my_msg_test:main"
        ],
    },
)

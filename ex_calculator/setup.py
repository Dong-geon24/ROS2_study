from setuptools import find_packages, setup
import os
import glob 

package_name = 'ex_calculator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
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
            'argument = ex_calculator.arithmetic.argument:main',
            'operator = ex_calculator.arithmetic.operator:main',
            'calculator = ex_calculator.calculator.main:main',
            'checker = ex_calculator.checker.main:main',
        ],
    },
)

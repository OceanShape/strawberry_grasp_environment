from setuptools import setup
import os
from glob import glob

package_name = 'e0509_gripper_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'curobo'),
            glob('config/curobo/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OceanShape',
    maintainer_email='gjxosdid1@gmail.com',
    description='cuRobo robot config for Doosan e0509 + RH-P12-RN gripper (data-only package)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

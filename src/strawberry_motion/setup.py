from setuptools import setup
import os
from glob import glob

package_name = 'strawberry_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.execution'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OceanShape',
    maintainer_email='gjxosdid1@gmail.com',
    description='Strawberry harvest motion planning — scan executor and safety modules',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # TODO: 노드 진입점 추가 (main() 함수 확인 후)
            # 'scan_executor_node = strawberry_motion.execution.scan_executor_node:main',
        ],
    },
)

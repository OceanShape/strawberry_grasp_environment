from setuptools import find_packages, setup

package_name = 'strawberry_sim_core'

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
    maintainer='sun',
    maintainer_email='oceanshape700@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fake_vision_node = strawberry_sim_core.fake_vision_node:main',
            'sim_executor_bridge_node = strawberry_sim_core.sim_executor_bridge_node:main',
            'status_monitor_node = strawberry_sim_core.status_monitor_node:main',
        ],
    },
)

from setuptools import find_packages, setup


package_name = 'slam_scratch'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_scratch.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='A self-contained live occupancy-grid SLAM demonstration.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sensor_talker = slam_scratch.sensor_talker:main',
            'scratch_slam = slam_scratch.scratch_slam:main',
            'map_listener = slam_scratch.map_listener:main',
        ],
    },
)

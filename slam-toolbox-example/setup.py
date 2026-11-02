from setuptools import find_packages, setup

package_name = 'slam_toolbox_example'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_toolbox_example.launch.py']),
        ('share/' + package_name + '/config', ['config/slam_toolbox.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='A synthetic sensor, SLAM Toolbox, and map listener demo.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sensor_talker = slam_toolbox_example.sensor_talker:main',
            'map_listener = slam_toolbox_example.map_listener:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'rviz_dummy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rviz_dummy.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Simple dummy ROS data publisher for the repository RViz config.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'talker = rviz_dummy.talker:main',
        ],
    },
)

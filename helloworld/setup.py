from setuptools import find_packages, setup

package_name = 'helloworld'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/helloworld.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='A small ROS 2 publisher/subscriber demo.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'talker = helloworld.talker:main',
            'listener = helloworld.listener:main',
        ],
    },
)

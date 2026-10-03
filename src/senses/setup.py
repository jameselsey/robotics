from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'senses'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/' + package_name + '/resource', glob('senses/resource/*')),
    ],
    install_requires=['setuptools', 'ament_index_python'],
    zip_safe=True,
    maintainer='James Elsey',
    maintainer_email='james.elsey@gmail.com',
    description='Package containing sensory and cognitive nodes for the robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_agent = senses.voice_agent:main',
            'room_markers = senses.room_markers:main',
            'joystick_voice_control = senses.joystick_voice_control:main',
        ],
    },
    include_package_data=True,
)

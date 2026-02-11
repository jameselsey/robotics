from setuptools import setup
from glob import glob
import os

package_name = 'senses'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/' + package_name + '/resource', glob('senses/resource/*')),
    ],
    install_requires=['setuptools', 'ament_index_python'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Package containing sensory and cognitive nodes for the robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eyes = senses.eyes:main',
            'ears = senses.ears:main',
            'mouth = senses.mouth:main',
            'brain = senses.brain:main',
            'screen = senses.screen:main',
        ],
    },
    package_data={
        'senses': ['resource/*.ppn'],
    },
    include_package_data=True,
)

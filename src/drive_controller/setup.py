from setuptools import find_packages, setup

package_name = 'drive_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='James Elsey',
    maintainer_email='jelsey@example.com',
    description='Drives motors using /cmd_vel and GPIO',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drive_controller = drive_controller.drive_controller_node:main'
        ],
    },
)

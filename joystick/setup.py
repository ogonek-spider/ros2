from setuptools import find_packages, setup

package_name = 'joystick'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'hidapi'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 Node for GuliKit Controller XW handling HID Events',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_node = joystick.joystick_node:main'
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'spider_pid_tuning'

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
    maintainer='alarin',
    maintainer_email='me@alarin.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'singen = spider_pid_tuning.sin_position_generator:main',
            'multigen = spider_pid_tuning.multi_position_generator:main',
        ],
    },
)

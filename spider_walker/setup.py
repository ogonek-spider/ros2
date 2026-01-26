import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'spider_walker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.[pP][yY]'))),
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
            'walker = spider_walker.spider_walker:main',
            'ik = spider_walker.spider_inversekinematics_test:main',
            'custom_ik = spider_walker.spider_custom_ik:main',
        ],
    },
)

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'digit_relay'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dff',
    maintainer_email='daffaaufafif@gmail.com',
    description='ROS 2 Python package for a distributed digit relay.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
<<<<<<< Updated upstream
    'console_scripts': [
        'digit_relay_member_01 = digit_relay.member_01:main',
        'digit_relay_member_02 = digit_relay.member_02:main',
        'digit_relay_member_03 = digit_relay.member_03:main',
        'digit_relay_member_04 = digit_relay.member_04:main',
        'digit_relay_member_05 = digit_relay.member_05:main',
        'digit_relay_member_06 = digit_relay.member_06:main',
        'digit_relay_member_07 = digit_relay.member_07:main',
        ## 'digit_relay_member_08 = digit_relay.member_08:main',
        'digit_relay_member_09 = digit_relay.member_09:main',
=======
        'console_scripts': [
            'digit_relay_member_01 = digit_relay.member_01:main',
            'digit_relay_member_08 = digit_relay.member_08:main',
>>>>>>> Stashed changes
        ],
    },
)

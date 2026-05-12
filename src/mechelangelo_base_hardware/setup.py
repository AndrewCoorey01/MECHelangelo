from glob import glob
from setuptools import setup

package_name = 'mechelangelo_base_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Coorey',
    maintainer_email='andrew.g.coorey@student.uts.edu.au',
    description='MECHelangelo real base driver for Raspberry Pi motor control and wheel encoders.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_driver = mechelangelo_base_driver.base_driver:main',
        ],
    },
)
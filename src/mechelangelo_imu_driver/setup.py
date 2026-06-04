from glob import glob
from setuptools import setup

package_name = 'mechelangelo_imu_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, 'sense_hat'],
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
            glob('config/*.yaml') + glob('config/*.ini')
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.py')
        ),
        # sense_hat text assets (needed for LED display, bundled for completeness)
        (
            'lib/python3/dist-packages/sense_hat',
            glob('sense_hat/*.png') + glob('sense_hat/*.txt')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andy',
    maintainer_email='andrewcoorey13@gmail.com',
    description='MECHelangelo Sense HAT IMU driver',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_driver = mechelangelo_imu_driver.imu_driver:main',
        ],
    },
)

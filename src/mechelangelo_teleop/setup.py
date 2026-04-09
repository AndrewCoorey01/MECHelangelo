from setuptools import setup

package_name = 'mechelangelo_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Coorey',
    maintainer_email='andrewcoorey13@gmail.com',
    description='Keyboard teleoperation for MECHelangelo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = mechelangelo_teleop.teleop_keyboard:main',
        ],
    },
)
from setuptools import setup
from glob import glob

package_name = 'luci_basic_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='josh',
    maintainer_email='josh@luci.com',
    description='An example node that can be used to drive LUCI using the arrow keys on your keyboard. ctrl+c or q to terminate. Compatible with Linux.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control_node = luci_basic_teleop.keyboard_control_node:main',
            'controller_control_node = luci_basic_teleop.controller_control_node:main',
            'luci_override_node = luci_basic_teleop.luci_override_node:main',
        ],
    },
)

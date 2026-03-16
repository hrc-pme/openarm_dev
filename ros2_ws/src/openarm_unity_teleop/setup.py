from setuptools import find_packages, setup

package_name = 'openarm_unity_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ken',
    maintainer_email='ken@todo.todo',
    description='Unity VR to OpenArm Joint control adapter node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unity_adapter_node = openarm_unity_teleop.unity_adapter_node:main'
        ],
    },
)

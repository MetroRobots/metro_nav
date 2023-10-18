from setuptools import find_packages, setup

package_name = 'metro_nav_demo_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David V. Lu!!',
    maintainer_email='davidvlu@gmail.com',
    description='Some utilities for testing out different parts of navigation',
    license='BSD 3-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner = metro_nav_demo_utils.global_planner:main',
            'odom_to_tf = metro_nav_demo_utils.odom_to_tf:main',
            'simple_exec = metro_nav_demo_utils.simple_exec:main',
        ]
    },
)

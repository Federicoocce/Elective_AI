from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_project' # This should be your actual package name

setup(
    name=package_name,
    version='0.0.1', # A default valid version, keep consistent with package.xml
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # If you have launch files, uncomment and adjust:
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # If you have config files, uncomment and adjust:
        # (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'nav2_msgs',
        'geometry_msgs',
        'action_msgs',
        'tf2_ros',
        'networkx',
        'groq',
        'python-dotenv',
        # Add any other existing install_requires from your project
        ],
    zip_safe=True,
    maintainer='Your Name', # TODO: Update this
    maintainer_email='user@todo.todo', # TODO: Update this
    description='The my_project package description. TODO: Update this.', # TODO: Update this
    license='Apache License 2.0', # TODO: Update or confirm your license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # If you have existing entry points, keep them here. For example:
            # 'my_existing_node = my_project.my_existing_module:main_function',
            'mall_controller = my_project.scripts.main_robot_controller:main',
        ],
    },
)

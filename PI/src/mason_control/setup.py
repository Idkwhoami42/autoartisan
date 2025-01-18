from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mason_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all config files.
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', "mason_description", "description", "urdf"), glob('urdf/*.xacro')),
        (os.path.join('share', "mason_description", "description", "meshes"), glob('meshes/*.STL')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lsitanyi',
    maintainer_email='lsitanyi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

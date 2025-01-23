from setuptools import find_packages, setup

package_name = 'mason_flask'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'flask', 'cv_bridge', 'numpy', 'cv2'],
    zip_safe=True,
    maintainer='neel',
    maintainer_email='N.Lodha-1@student.tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flask_node = mason_flask.flask_server:main',
        ],
    },
)

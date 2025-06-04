from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ovcs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        (
            os.path.join('share', package_name, 'urdf'), ["urdf/colors.xacro"]
        ),
        (
            os.path.join('share', package_name, 'urdf/test'),
            glob('urdf/test/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Spin42',
    maintainer_email='info@spin42.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_camera_node = ovcs.stereo_camera_publisher:main',
        ],
        'ros2_components': [
            'StereoCameraPublisher = ovcs.stereo_camera_publisher:StereoCameraPublisher',
        ],
    },
)

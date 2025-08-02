from setuptools import find_packages, setup
import glob
import os

package_name = 'yolo_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'weights'), glob.glob('weights/*.pt')),
    ],
    install_requires=[
        'setuptools',
        'torch>=1.9.0',
        'torchvision',
        'opencv-python',
        'numpy',
        'Pillow'
    ],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='fallthrive@outlook.com',
    description='YOLO-based object detection for UAV',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_scene = yolo_detection.yolo_scene:main'
        ],
    },
)
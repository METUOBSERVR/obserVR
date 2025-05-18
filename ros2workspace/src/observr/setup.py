from setuptools import find_packages, setup
import os

package_name = 'observr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'),
         ['observr/data/calibrationL.calib']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='observr',
    maintainer_email='observr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'imu_acquire = observr.imu_ros2_integrated:main',
        'cam_acquire = observr.camera_ros2_wrapper:main',
        "egomotion_solver = observr.run_egomotion:main",
        'stream_view = observr.camera_stream:main',
        'rtsp_bridge = observr.rtsp_bridge_rightcam:main',
        'tcp_bridge = observr.tcp_imucam:main'
        ],
    },
)

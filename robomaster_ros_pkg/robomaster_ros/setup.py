from setuptools import setup
import os
import glob

package_name = 'robomaster_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.modules'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.xml')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob.glob('config/*'))
    ],
    install_requires=['setuptools', 'numpy', 'numpy-quaternion', 'pyyaml', 'robomaster',
                      'opencv-python'],
    zip_safe=True,
    maintainer='Jerome Guzzi',
    maintainer_email='jerome@idsia.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robomaster_driver = robomaster_ros.robomaster_driver:main',
            'h264_decoder = robomaster_ros.decompress_h264:main',
            'play_audio = robomaster_ros.play_audio:main',
            'play_opus = robomaster_ros.play_audio_opus:main',
            'display_battery = robomaster_ros.display_battery:main',
            'connect = robomaster_ros.connect:main',
            'discover = robomaster_ros.discover:main',
            'keyboard_teleop = robomaster_ros.keyboard_teleop:main',
            'joy_servo_angle_teleop = robomaster_ros.joy_servo_angle_teleop:main',
            'new_discover = robomaster_ros.new_discover:main',
            'episode_recorder = robomaster_ros.episode_recorder:main',
        ],
    },
)

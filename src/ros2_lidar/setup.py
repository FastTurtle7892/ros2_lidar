from setuptools import setup
import os
from glob import glob

package_name = 'ros2_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['pi_car'],  # pi_car.py를 모듈로 등록
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('*.launch.py')), # 런치 파일
        (os.path.join('share', package_name), glob('*.urdf')),      # URDF 파일
        (os.path.join('share', package_name), glob('*.yaml')),      # 설정 파일
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='ROS 2 Lidar Project',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pi_car = pi_car:main', # pi_car 명령어 실행 시 main 함수 연결
        ],
    },
)
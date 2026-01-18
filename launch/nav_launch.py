import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_lidar')
    
    # [수정됨] 패키지 내부의 maps 폴더에서 자동으로 찾기
    map_file_path = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    # 1. 라이다 센서 실행
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rplidar_ros'), '/launch/rplidar.launch.py'
        ]),
        launch_arguments={'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser'}.items()
    )

    # 2. RF2O 오도메트리
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0
        }],
    )

    # 3. Nav2 내비게이션
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'), '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'map': map_file_path,   # <--- 위에서 자동 설정된 경로 사용
            'use_sim_time': 'False',
            'autostart': 'True',
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml')
        }.items()
    )

    # 4. 로봇 모델 (URDF)
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'picar.urdf')
    try:
        with open(urdf_file_path, 'r') as infp:
            robot_desc = infp.read()
    except FileNotFoundError:
        robot_desc = ""

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # 5. Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': False}],
    )

    # 6. 모터 드라이버
    motor_driver = Node(
        package='ros2_lidar',
        executable='pi_car.py',
        output='screen',
    )

    return LaunchDescription([
        rplidar_launch,
        rf2o_node,
        nav2_launch,
        robot_state_publisher,
        joint_state_publisher,
        motor_driver,
    ])

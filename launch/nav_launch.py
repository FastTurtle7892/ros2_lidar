import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_lidar')
    
    # 맵 파일 경로 설정
    map_file_path = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    # 1. 라이다 실행 (Hardware)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rplidar_ros'), '/launch/rplidar.launch.py'
        ]),
        launch_arguments={'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser'}.items()
    )

    # 2. RF2O Laser Odometry 실행 (Localization Base)
    rf2o_node = Node(
        package='ros2_lidar',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0
        }],
    )

    # 3. [핵심 변경] Nav2 Bringup 실행 (Navigation)
    # SLAM 대신 맵을 불러오고 경로를 생성하는 Nav2를 실행합니다.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'), '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'map': map_file_path,
            'use_sim_time': 'False', # 실물 로봇이므로 False
			'autostart': 'True'
            # 'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml') # 파라미터 파일이 있다면 주석 해제
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

    # 5. 모터 드라이버
    motor_driver = Node(
        package='ros2_lidar',
        executable='pi_car.py',
        output='screen'
    )

    # 6. Foxglove Bridge (모니터링)
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'topic_whitelist': ['.*']
        }]
    )

    return LaunchDescription([
        rplidar_launch,
        rf2o_node,
        nav2_launch,   # SLAM 대신 이게 들어감
        robot_state_publisher,
        motor_driver,
        foxglove_bridge
    ])

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_lidar')
    
    # 맵 파일 경로 (my_map.yaml)
    map_file_path = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    # 1. 라이다 센서 실행
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rplidar_ros'), '/launch/rplidar.launch.py'
        ]),
        launch_arguments={'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser'}.items()
    )

    # 2. RF2O 오도메트리 (위치 추정 기초)
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

    # 3. Nav2 내비게이션 (맵 서버, 경로 탐색)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'), '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'map': map_file_path,
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

    # 5. Joint State Publisher (바퀴 연결 오류 해결)
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
        #remappings=[('/cmd_vel', '/cmd_vel_nav')]  # 만약 Nav2가 cmd_vel_nav를 쓴다면 추가
    )

    # 7. RViz (라즈베리 파이에서는 주석 처리)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    # )

    # === [중요] 모든 노드를 리스트에 포함해야 합니다 ===
    return LaunchDescription([
        rplidar_launch,        # 라이다
        rf2o_node,             # 오도메트리
        nav2_launch,           # 내비게이션(맵)
        robot_state_publisher, # 로봇 모델
        joint_state_publisher, # 관절 상태 (바퀴)
        motor_driver,          # 모터
        # rviz_node            # 화면 (PC에서 실행)
    ])

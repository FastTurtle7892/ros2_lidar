import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_lidar')
    
    # =========================================
    # 1. 하드웨어 및 필수 노드 실행 (공통 영역)
    # =========================================

    # 1-1. 라이다 (RPLidar)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rplidar_ros'), '/launch/rplidar.launch.py'
        ]),
        launch_arguments={'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser'}.items()
    )

    # 1-2. 오도메트리 (RF2O) - 위치 추정
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

    # 1-3. 로봇 모델 (URDF) - TF 트리 생성
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

    # 1-4. 바퀴 관절 상태 (Joint State)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # 1-5. 모터 드라이버
    motor_driver = Node(
        package='ros2_lidar',
        executable='pi_car.py',
        output='screen'
    )

    # =========================================
    # 2. SLAM (지도 제작) 기능
    # =========================================
    
    slam_config_path = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'), '/launch/online_async_launch.py'
        ]),
        launch_arguments={'slam_params_file': slam_config_path}.items()
    )

    return LaunchDescription([
        rplidar_launch,
        rf2o_node,
        robot_state_publisher,
        joint_state_publisher,
        motor_driver,
        slam_launch
    ])

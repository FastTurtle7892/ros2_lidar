import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로를 자동으로 찾습니다.
    pkg_share = get_package_share_directory('ros2_lidar')
    
    # 1. 라이다 실행
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rplidar_ros'), '/launch/rplidar.launch.py'
        ]),
        launch_arguments={'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser'}.items()
    )

    # 2. RF2O Laser Odometry 실행
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

    # 3. SLAM (설정 파일 경로 자동 탐색)
    # CMakeLists.txt에서 config 폴더로 설치하도록 설정했습니다.
    slam_config_path = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'), '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': slam_config_path
        }.items()
    )

    # 4. 로봇 모델 (URDF 경로 자동 탐색)
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'picar.urdf')
    
    # 파일을 읽어서 파라미터로 넘깁니다.
    try:
        with open(urdf_file_path, 'r') as infp:
            robot_desc = infp.read()
    except FileNotFoundError:
        # 혹시 파일이 없을 경우를 대비해 로그를 남깁니다 (실행 시 에러 확인용)
        print(f"ERROR: URDF file not found at {urdf_file_path}")
        robot_desc = ""

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # 5. 모터 드라이버 (pi_car.py)
    motor_driver = Node(
        package='ros2_lidar',
        executable='pi_car.py',
        output='screen'
    )

    # 6. Foxglove Bridge (윈도우 연결용)
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
        slam_launch,
        robot_state_publisher,
        motor_driver,
        foxglove_bridge
    ])

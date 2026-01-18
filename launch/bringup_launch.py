import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_lidar')
    
    # 1. RPLidar 실행 (A1 모델 기준)
    # rplidar_ros 패키지 내의 런치 파일 사용
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rplidar_ros'), '/launch/rplidar_a1_launch.py'
        ]),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser_frame'
        }.items()
    )

    # 1-1. TF (base_link -> laser_frame)
    # 라이다 위치를 알려주는 정적 TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.0', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    # 2. RF2O Laser Odometry (외부 패키지 사용)
    # [수정 포인트] package='rf2o_laser_odometry'
    rf2o_node = Node(
        package='rf2o_laser_odometry',
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

    # 3. SLAM (Sim Time 꺼짐)
    slam_config_path = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'), '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': slam_config_path,
            'use_sim_time': 'false'
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

    # 6. RViz2 실행 (Foxglove 대신 실행)
    # 저장된 rviz 설정 파일이 있다면 경로를 지정해주세요. (없으면 기본 실행)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', os.path.join(pkg_share, 'config', 'my_config.rviz')] 
    )

    return LaunchDescription([
        static_tf,
        rplidar_launch,
        rf2o_node,
        slam_launch,
        robot_state_publisher,
        motor_driver,
        rviz_node
    ])

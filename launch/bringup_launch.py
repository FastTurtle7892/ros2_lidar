import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_lidar')
    
    # 1. RPLidar 실행
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rplidar_ros'), '/launch/rplidar.launch.py'
        ]),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser'  # URDF의 링크 이름과 일치시킴
        }.items()
    )

    # [삭제됨] static_tf_pub_laser
    # 이유: URDF(picar.urdf)에 이미 laser_joint가 정의되어 있으므로, 
    # 여기서 또 발행하면 충돌이 나서 라이다가 안 보입니다. 과감히 삭제!

    # 2. RF2O Laser Odometry
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            # [수정됨] base_link -> base_footprint
            # 이유: URDF 상위 부모인 base_footprint를 움직여야 전체 로봇이 움직입니다.
            'base_frame_id': 'base_footprint', 
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0
        }],
    )

    # 3. SLAM
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

    # 4-1. Joint State Publisher (바퀴 보이게 하기 위해 필수)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # 5. 모터 드라이버
    motor_driver = Node(
        package='ros2_lidar',
        executable='pi_car.py',
        output='screen'
    )

    return LaunchDescription([
        # static_tf,  <-- 제거함
        rplidar_launch,
        rf2o_node,
        slam_launch,
        robot_state_publisher,
        joint_state_publisher,
        motor_driver
    ])

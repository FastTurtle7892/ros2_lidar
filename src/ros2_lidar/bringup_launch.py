import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 라이다 실행
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rplidar_ros'), '/launch/rplidar.launch.py'
        ]),
        launch_arguments={'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser'}.items()
    )

    # [추가] Lidar Odometry 실행 (휠 오도메트리 대신 사용)
    # package 이름은 package.xml에 명시된 'lidar_odometry'를 사용합니다.
    lidar_odometry_node = Node(
        package='lidar_odometry',
        executable='lidar_odometry_node',
        output='screen',
        parameters=[{
            'scan_topic_name': '/scan',
            'odom_topic_name': '/odom'
        }]
    )

    # 2. SLAM (설정 파일 적용!)
    # 설정 파일의 절대 경로를 만듭니다.
    params_file = os.path.join(os.getenv('HOME'), 'my_robot', 'slam_params.yaml')
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'), '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': params_file
        }.items()
    )

    # 3. 로봇 모델
    urdf_file = 'picar.urdf'
    # 주의: urdf_file 경로가 현재 실행 위치에 있어야 합니다.
    # 만약 에러가 난다면 절대 경로로 수정하거나 launch 파일과 같은 폴더에 두세요.
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # 4. 모터 드라이버 (pi_car.py)
    # 주의: 이 파일도 경로가 맞아야 실행됩니다.
    motor_driver = ExecuteProcess(
        cmd=['python3', 'pi_car.py'],
        output='screen'
    )

    # 5. Foxglove Bridge (윈도우 연결용)
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
        lidar_odometry_node, # 추가된 노드 등록
        slam_launch,
        robot_state_publisher,
        motor_driver,
        foxglove_bridge
    ])
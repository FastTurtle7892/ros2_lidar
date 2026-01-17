import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    package_name = 'ros2_lidar'
    pkg_path = get_package_share_directory(package_name)

    # 1. World 파일 설정
    # (주의: Gazebo Classic용 .world 파일은 호환되지 않을 수 있습니다. 
    #  일단 빈 월드나 기본 월드로 테스트 후, SDF 파일을 새로 만드시는 것을 추천합니다.)
    #  여기서는 Gazebo 기본 빈 월드를 실행합니다.
    
    # 2. URDF 설정
    xacro_file = os.path.join(pkg_path, 'urdf', 'picar.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # 3. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 4. Gazebo Harmonic 실행 (gz_sim)
    # -r: 실행 시 자동 시작, empty.sdf: 빈 월드
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 5. 로봇 스폰 (Spawn)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', 
                   '-name', 'my_bot', 
                   '-z', '0.1'],
        output='screen'
    )

    # 6. ROS-Gazebo Bridge (필수!)
    # ROS의 /cmd_vel을 Gazebo로, Gazebo의 /camera/image_raw를 ROS로 연결
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/my_bot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry' # Odom 필요 시
        ],
        output='screen'
    )

    # RViz2 실행
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn,
        bridge,
        rviz_node
    ])

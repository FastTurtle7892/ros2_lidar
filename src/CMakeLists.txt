cmake_minimum_required(VERSION 3.8)
project(ros2_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)  # 파이썬 사용을 위해 필요
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 2. C++ 오도메트리 노드 빌드 (src 폴더 안의 cpp 파일들)
add_executable(rf2o_laser_odometry_node
  src/CLaserOdometry2D.cpp
  src/CLaserOdometry2DNode.cpp
)
target_include_directories(rf2o_laser_odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rf2o_laser_odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 3. 설치 설정 (Install)

# 3-1. C++ 실행 파일 설치
install(TARGETS rf2o_laser_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

# 3-2. Python 스크립트 (pi_car.py) 설치
# scripts 폴더 안에 있는 pi_car.py를 실행 가능하게 만듭니다.
install(PROGRAMS
  scripts/pi_car.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3-3. 폴더들 설치 (launch, urdf, config 등)
install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.5)
project(ros2_lidar)

# 1. C++ 컴파일 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# 3. Include 디렉토리 설정
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# 4. C++ 노드 빌드 (rf2o_laser_odometry 로직)
add_executable(rf2o_laser_odometry_node
  src/CLaserOdometry2DNode.cpp
  src/CLaserOdometry2D.cpp
)

ament_target_dependencies(rf2o_laser_odometry_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  Eigen3
)

# 5. 설치 규칙 (Install Rules)

# 5-1. C++ 실행 파일 설치
install(TARGETS rf2o_laser_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

# 5-2. Python 스크립트 설치 (scripts/pi_car.py)
install(PROGRAMS
  scripts/pi_car.py
  DESTINATION lib/${PROJECT_NAME}
)

# 5-3. Launch 및 URDF 폴더 설치
install(DIRECTORY
  launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# 5-4. SLAM 설정 파일 설치 (루트에 있는 slam_params.yaml을 config 폴더로 설치)
install(FILES
  slam_params.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# 5-5. Map 파일이 있다면 설치 (선택 사항)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/maps")
  install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
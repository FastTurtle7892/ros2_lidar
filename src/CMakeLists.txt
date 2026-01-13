# cmake_minimum_required(VERSION 3.5)
# project(rf2o_laser_odometry)


# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf2  REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)

# ## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED)
# find_package(eigen3_cmake_module REQUIRED)
# find_package(Eigen3 REQUIRED)


# set(EXECUTABLE_NAME "rf2o_laser_odometry_node")

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# add_executable(${EXECUTABLE_NAME} 
#     src/CLaserOdometry2DNode.cpp
#     src/CLaserOdometry2D.cpp)

# include_directories(
#   include include/
# )

# include_directories(  
#   ${Boost_INCLUDE_DIRS}
#   ##${EIGEN3_INCLUDE_DIRS}
# )

# set(DEPENDENCIES
#   "rclcpp"           
#   "std_msgs"         
#   "geometry_msgs" 
#   "tf2_geometry_msgs"
#   "tf2"
#   "tf2_ros"
#   "sensor_msgs"
#   "nav_msgs"
#   "eigen3_cmake_module"
# )

# ##target_include_directories(${PROJECT_NAME} PUBLIC ${Eigen3_INCLUDE_DIRS})
# ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})

# ## Declare a cpp library
# add_library(${PROJECT_NAME} src/CLaserOdometry2D.cpp)

# ## Declare a cpp executable
# ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

# install(TARGETS ${EXECUTABLE_NAME}
#   DESTINATION lib/${PROJECT_NAME})

# install(
#   DIRECTORY launch  
#   DESTINATION share/${PROJECT_NAME})

# ament_package()

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

# 5-2. Python 스크립트 설치 (scripts 폴더에 있는 pi_car.py)
install(PROGRAMS
  scripts/pi_car.py
  DESTINATION lib/${PROJECT_NAME}
)

# 5-3. 나머지 폴더들 설치 (launch, urdf, config, maps 등)
install(DIRECTORY
  launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# 설정 파일이나 맵 파일이 있다면 추가 (예: config, maps)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/maps")
  install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
endif()
# 루트에 있는 yaml 파일 등도 필요하면 share로 보냄 (여기서는 예시)
install(FILES slam_params.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
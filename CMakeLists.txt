cmake_minimum_required(VERSION 3.8)
project(ros2_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# [삭제됨] 기존 rf2o 빌드 관련 코드 (add_executable 등) 모두 삭제

# 1. Python 스크립트 설치 (pi_car.py)
install(PROGRAMS
  scripts/pi_car.py
  DESTINATION lib/${PROJECT_NAME}
)

# 2. 런치, 설정, 맵, URDF 폴더 설치
install(DIRECTORY
  launch
  urdf
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

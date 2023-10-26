cmake_minimum_required(VERSION 3.8)
project(pilz_fair_revpi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(shared_memory src/shared_memory.cpp)
ament_target_dependencies(shared_memory rclcpp geometry_msgs)
target_link_libraries(shared_memory rt)

install(TARGETS
  shared_memory
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY configs launch urdf maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/nav_waypoints_follower.py
  scripts/odom.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(robocross2023)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(rrtStar scripts/rrtStar.cpp)
ament_target_dependencies(rrtStar rclcpp std_msgs)

install(TARGETS
  rrtStar
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(
  PROGRAMS
  scripts/rrtStar.cpp
  scripts/create_path.py
  scripts/local_rrt_star.py
  scripts/global_rrt_star.py
  scripts/go_to_point.py
  scripts/car_in_global_map.py
  scripts/static_turtle_tf2_broadcaster.py
  scripts/transforms.py
  scripts/teleop.py
  scripts/create_local_cost_map.py
  scripts/create_local_cost_map_old.py
  scripts/create_local_cost_map_new.py
  scripts/test_r_auto.py
  scripts/spawn_global_map.py
  scripts/pcd_to_map.py
  scripts/spawn_robot.py
  scripts/surf.py
  scripts/calc_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    worlds
    models
    rviz
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()

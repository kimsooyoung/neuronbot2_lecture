cmake_minimum_required(VERSION 3.5)
project(neuronbot2_path_planning)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(custom_nav2_msgs REQUIRED)
find_package(custom_nav2_bt_navigator REQUIRED)
find_package(custom_nav2_rviz_plugins REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(test_node src/test.cpp)
ament_target_dependencies(test_node rclcpp custom_nav2_msgs)     # CHANGE

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/goal_pose_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/nav_to_pose_action_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz config map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

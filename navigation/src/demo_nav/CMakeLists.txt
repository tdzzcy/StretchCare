cmake_minimum_required(VERSION 3.8)
project(demo_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# include Cpp "include" directory
include_directories(include)

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

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY map DESTINATION share/${PROJECT_NAME})
install(DIRECTORY demo_nav DESTINATION share/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  demo_nav/simple_commander_demo.py
  demo_nav/detect_faces.py
  demo_nav/detection_node.py
  demo_nav/detection_ros_markers.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

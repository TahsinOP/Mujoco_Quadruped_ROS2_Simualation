cmake_minimum_required(VERSION 3.8)
project(waypoint_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

set(TARGET_DEPENDENCE
  ament_cmake
)


install(
  DIRECTORY config launch scripts 
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/waypoint_navigation.py
    scripts/gait_test.py
    scripts/trotting_gait.py 
    scripts/ik_2D.py
    scripts/ik_3D.py
    scripts/kinematics.py
    scripts/test.py
     # Replace with your actual script name(s)
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()





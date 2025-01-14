cmake_minimum_required(VERSION 3.8)
project(turtle2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(--coverage -O0 -g)
  link_libraries(--coverage)
endif()

add_library(turtle2_lib src/turtle2.cpp)
ament_target_dependencies(turtle2_lib rclcpp std_msgs geometry_msgs turtlesim)

add_executable(turtle2_node src/main.cpp)
target_link_libraries(turtle2_node turtle2_lib)
ament_target_dependencies(turtle2_node rclcpp std_msgs geometry_msgs turtlesim)

ament_add_gtest(test_turtle2 test/test_turtle2.cpp)
if(TARGET test_turtle2)
  target_link_libraries(test_turtle2 turtle2_lib)
  ament_target_dependencies(test_turtle2 rclcpp turtlesim)
endif()

install(TARGETS
  turtle2_node
  turtle2_lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ../turtle_control/turtle_control/leader.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(flight_stack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(flight_stack_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# include_directories(include)

add_executable(flight_core_node src/flight_core_node.cpp src/flight_core.cpp)
target_include_directories(flight_core_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(flight_core_node rclcpp px4_msgs flight_stack_msgs)
target_compile_features(flight_core_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS flight_core_node
  DESTINATION lib/${PROJECT_NAME})

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

# Python

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/pub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

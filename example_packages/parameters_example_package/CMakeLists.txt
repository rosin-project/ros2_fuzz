cmake_minimum_required (VERSION 3.5)

project (parameters_example_package)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set (CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set (CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options (-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package (ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package (example_interfaces REQUIRED)
find_package (std_msgs REQUIRED)

add_executable (server src/add_two_ints_server.cpp)
ament_target_dependencies (server rclcpp example_interfaces)

add_executable (client src/fuzz_target.cpp)
ament_target_dependencies (client rclcpp example_interfaces std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install (TARGETS
  server
  client
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package ()

cmake_minimum_required(VERSION 3.5)
project(automatic_fuzzing)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(example_interfaces REQUIRED)
find_package(tutorial_interfaces REQUIRED)        # CHANGE

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
  rclcpp #example_interfaces) CHANGE
         tutorial_interfaces) 

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp #example_interfaces) CHANGE
         tutorial_interfaces) 

# ADDED FOR FUZZING 
add_executable(fuzzer_device src/generated.cpp)
ament_target_dependencies(fuzzer_device rclcpp tutorial_interfaces) 

install(TARGETS
  server
  client
  fuzzer_device
  DESTINATION lib/${PROJECT_NAME})

ament_package()


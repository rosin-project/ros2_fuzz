cmake_minimum_required(VERSION 3.5)
project(client_service_example)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(example_interfaces REQUIRED)
find_package(tutorial_interfaces REQUIRED)        # CHANGE

add_executable(server src/add_three_ints_server.cpp)
ament_target_dependencies(server
  rclcpp #example_interfaces) CHANGE
         tutorial_interfaces) 

add_executable(client src/add_three_ints_client.cpp)
ament_target_dependencies(client
  rclcpp #example_interfaces) CHANGE
         tutorial_interfaces) 

# ADDED FOR FUZZING 
add_executable(generated_fuzzer src/add_three_ints_server_generated.cpp)
ament_target_dependencies(generated_fuzzer rclcpp tutorial_interfaces) 

install(TARGETS
  server
  client
  generated_fuzzer
  DESTINATION lib/${PROJECT_NAME})

ament_package()


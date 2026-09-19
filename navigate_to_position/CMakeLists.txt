cmake_minimum_required(VERSION 3.8)
project(navigate_to_position)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "action/NavigateToPosition.action")

install(PROGRAMS scripts/navigate_to_position_server.py
  DESTINATION lib/${PROJECT_NAME} RENAME navigate_to_position_server)
install(PROGRAMS scripts/navigate_to_position_client.py
  DESTINATION lib/${PROJECT_NAME} RENAME navigate_to_position_client)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

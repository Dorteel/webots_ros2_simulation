cmake_minimum_required(VERSION 3.8)
project(navigate_to_position)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "action/NavigateToPosition.action")

install(PROGRAMS scripts/navigate_to_position_server.py
  DESTINATION lib/${PROJECT_NAME} RENAME navigate_to_position_server)
install(PROGRAMS scripts/navigate_to_position_client.py
  DESTINATION lib/${PROJECT_NAME} RENAME navigate_to_position_client)
install(PROGRAMS scripts/follow_saved_path_client.py
  DESTINATION lib/${PROJECT_NAME} RENAME follow_saved_path)

ament_get_python_install_dir(PYTHON_DIR)
install(FILES scripts/client_api.py scripts/follow_saved_path_client.py
  DESTINATION ${PYTHON_DIR}/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()

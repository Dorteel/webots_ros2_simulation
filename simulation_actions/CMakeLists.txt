cmake_minimum_required(VERSION 3.8)
project(simulation_actions)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "action/Pick.action")

# Install the helper beside the generated action Python module.
ament_get_python_install_dir(PYTHON_DIR)
install(FILES python/pick_client.py DESTINATION ${PYTHON_DIR}/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()

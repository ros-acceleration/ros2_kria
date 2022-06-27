cmake_minimum_required(VERSION 3.5)
project(ros2_kria_power)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tracetools REQUIRED)
find_package(tracetools_acceleration REQUIRED)
find_package(rcutils REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

if (ROS_VITIS)
  find_package(vitis_common REQUIRED)
  find_package(ros2_kria_msgs REQUIRED)

  add_library(power_component SHARED
  src/power_component.cpp)
  target_compile_definitions(power_component
    PRIVATE "COMPOSITION_BUILDING_DLL")
  ament_target_dependencies(power_component
    "rclcpp"
    "rclcpp_components"
    "ros2_kria_msgs"
    "vitis_common"
    "tracetools"
    "tracetools_acceleration")
  rclcpp_components_register_nodes(power_component "composition::Power")
  set(node_plugins "${node_plugins}composition::Power;$<TARGET_FILE:power_component>\n")

  # install components
  install(TARGETS
    power_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)


  add_executable(composed src/composed.cpp)
  target_link_libraries(composed power_component)
  ament_target_dependencies(composed rclcpp class_loader)
  # If the interfaces are created in the same package that utilizes them,
  # you'll need to link against them using the ``rosidl_target_interfaces`` macro.
  #
  # rosidl_target_interfaces(composed
  #       ${PROJECT_NAME} "rosidl_typesupport_cpp")

  add_executable(power_node src/power_node.cpp)
  target_link_libraries(power_node power_component)
  ament_target_dependencies(power_node rclcpp class_loader)

  # install executables
  install(TARGETS
    composed
    power_node
    DESTINATION lib/${PROJECT_NAME}
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  # Install launch files.
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()  # ROS_VITIS

ament_package()

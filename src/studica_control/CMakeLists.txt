cmake_minimum_required(VERSION 3.8)
project(studica_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED) # for rclcpp_components
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} # <- SRV files (srv, msg, action)
  "msg/InitializeParams.msg"
  "srv/ControlImu.srv"
  "srv/SetData.srv"
)

include_directories( # <- #include <"">
  include 
  /usr/local/include/vmxpi 
  /usr/local/include/studica_drivers
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
 
set(studica_control_HDRS
  include/studica_control/device.h
  include/studica_control/analog_input.h
  include/studica_control/sharp_sensor_node.h
  # components
  include/studica_control/encoder_component.h
  include/studica_control/servo_component.h
  include/studica_control/ultrasonic_component.h
)

set(studica_control_SRCS # <- source files
  

  src/dynamic_publisher.cpp

  src/analog_input.cpp
  src/sharp_sensor_node.cpp





  
  # DRIVERS
  # src/drivers/encoder.cpp
  # src/drivers/servo.cpp
)

set(dependencies "ament_index_cpp" "sensor_msgs" "rclcpp_components" "rclcpp" "rclcpp_action" "std_msgs" "std_srvs")


# COMPONENTS
set(node_plugins "")

# FIND DRIVER LIBRARY, if not found make sure to install drivers
find_library(STUDICA_DRIVER_LIB studica_drivers PATHS /usr/local/lib/studica_drivers)

function(COMPILE_COMPONENT component_name)
  add_library(${component_name} SHARED src/components/${component_name}.cpp)
  target_compile_definitions(${component_name} PRIVATE "COMPOSITION_BUILDING_DLL")
  ament_target_dependencies(${component_name} ${dependencies})
  rclcpp_components_register_nodes(${component_name} "studica_control::${component_name}")
  set(node_plugins "${node_plugins}studica_control::${component_name};$<TARGET_FILE:${component_name}>\n")
  target_link_libraries(${component_name} ${STUDICA_DRIVER_LIB})
endfunction()

COMPILE_COMPONENT(encoder_component)
COMPILE_COMPONENT(servo_component)
COMPILE_COMPONENT(ultrasonic_component)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS # <- Install Components
  encoder_component
  servo_component
  ultrasonic_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# EXECUTABLES
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


# add_executable(studica_control_node src/studica_control_node.cpp ${studica_control_SRCS} ${studica_control_HDRS}) 
# target_link_libraries(studica_control_node 
#   encoder_component
#   servo_component
#   /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so
#   )
# ament_target_dependencies(studica_control_node ${dependencies})
# target_link_libraries(studica_control_node "${cpp_typesupport_target}")

add_executable(manual_composition 
  src/manual_composition.cpp)
target_link_libraries(manual_composition 
  encoder_component
  servo_component
  ultrasonic_component
  /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so
  /usr/local/lib/studica_drivers/libstudica_drivers.so
  # ${STUDICA_DRIVER_LIB}
)
ament_target_dependencies(manual_composition ${dependencies})
target_link_libraries(manual_composition "${cpp_typesupport_target}")

install(TARGETS # <- Executable targets
  # studica_control_node 
  manual_composition
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
set(CMAKE_INSTALL_RPATH "/usr/local/lib/studica_drivers")

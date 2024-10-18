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

include_directories(
  include 
  /usr/local/include/vmxpi src/drivers
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
) # <- #include <"">


  
# ----------------------------------------------------------------------------------
set(studica_control_SRCS # <- source files
  src/studica_control_node.cpp
  src/imu_driver_node.cpp
  src/dynamic_publisher.cpp
  src/ultrasonic.cpp
  src/analog_input.cpp
  src/sharp_sensor_node.cpp
  src/DIOPin.cpp
  src/cobra_sensor_node.cpp
  src/servo.cpp
  src/titan.cpp
  src/i2c.cpp
  
  src/drivers/
  
)
  
set(studica_control_HDRS # i don't think this is necessary
  include/studica_control/device.h
  include/studica_control/studica_control_node.h
  include/studica_control/imu_driver_node.h
  include/studica_control/ultrasonic.h
  include/studica_control/i2c.h
  include/studica_control/cobra_sensor_node.h
  include/studica_control/analog_input.h
  include/studica_control/sharp_sensor_node.h
  include/studica_control/titan.h
  include/studica_control/encoder_component.h
)
set(dependencies "ament_index_cpp" "sensor_msgs" "rclcpp_components" "rclcpp" "rclcpp_action" "std_msgs" "std_srvs")
  # ----------------------------------------------------------------------------------
  # COMPONENTS
set(node_plugins "")

add_library(encoder_component SHARED src/encoder_component.cpp)
target_compile_definitions(encoder_component PRIVATE "COMPOSITION_BUILDING_DLL")
# target_include_directories(encoder_component PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
#   $<INSTALL_INTERFACE:include>
# )
ament_target_dependencies(encoder_component ${dependencies})
rclcpp_components_register_nodes(encoder_component "studica_control::Encoder")
set(node_plugins "${node_plugins}studica_control::Encoder;$<TARGET_FILE:encoder_component>\n")

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


add_executable(studica_control_node ${studica_control_SRCS} ${studica_control_HDRS})
target_link_libraries(studica_control_node 
  encoder_component
  /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so)
ament_target_dependencies(studica_control_node ${dependencies})
target_link_libraries(studica_control_node "${cpp_typesupport_target}")

add_executable(tester src/test.cpp)
ament_target_dependencies(tester ${dependencies})
target_link_libraries(tester "${cpp_typesupport_target}")

install(TARGETS # <- Install Components
  encoder_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS # <- Executable targets
  studica_control_node 
  tester  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

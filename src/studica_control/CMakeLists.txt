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
  # components
  include/studica_control/encoder_component.h
  include/studica_control/servo_component.h
  include/studica_control/ultrasonic_component.h
  include/studica_control/sharp_component.h
  include/studica_control/cobra_component.h
)

set(studica_control_SRCS # <- source files
  

  # src/dynamic_publisher.cpp

  # src/analog_input.cpp
  # src/sharp_sensor_node.cpp





  
  # DRIVERS
  # src/drivers/encoder.cpp
  # src/drivers/servo.cpp
)

set(dependencies "ament_index_cpp" "sensor_msgs" "rclcpp_components" "rclcpp" "rclcpp_action" "std_msgs" "std_srvs")


# COMPONENTS
add_library(encoder_component SHARED src/components/encoder_component.cpp)
ament_target_dependencies(encoder_component ${dependencies})
rclcpp_components_register_nodes(encoder_component "studica_control::encoder")

add_library(servo_component SHARED src/components/servo_component.cpp)
ament_target_dependencies(servo_component ${dependencies})
rclcpp_components_register_nodes(servo_component "studica_control::Servo")

add_library(ultrasonic_component SHARED src/components/ultrasonic_component.cpp)
ament_target_dependencies(ultrasonic_component ${dependencies})
rclcpp_components_register_nodes(ultrasonic_component "studica_control::ultrasonic")

add_library(sharp_component SHARED src/components/sharp_component.cpp)
ament_target_dependencies(sharp_component ${dependencies})
rclcpp_components_register_nodes(sharp_component "studica_control::sharp")

add_library(cobra_component SHARED src/components/cobra_component.cpp)
ament_target_dependencies(cobra_component ${dependencies})
rclcpp_components_register_nodes(cobra_component "studica_control::cobra")

add_library(titan_component SHARED src/components/titan_component.cpp)
ament_target_dependencies(titan_component ${dependencies})
rclcpp_components_register_nodes(titan_component "studica_control::titan")


install(TARGETS # <- Install Components
  encoder_component
  servo_component
  ultrasonic_component
  sharp_component
  cobra_component
  titan_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


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
  sharp_component
  cobra_component
  titan_component
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

# tester for drivers
add_executable(comp_test 
  src/comp_test.cpp)
target_link_libraries(comp_test 
  encoder_component
  servo_component
  sharp_component
  cobra_component
  titan_component
  /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so
  /usr/local/lib/studica_drivers/libstudica_drivers.so
  # ${STUDICA_DRIVER_LIB}
)
ament_target_dependencies(comp_test "rclcpp")

install(TARGETS # <- Executable targets
  # studica_control_node 
  comp_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
set(CMAKE_INSTALL_RPATH "/usr/local/lib/studica_drivers")
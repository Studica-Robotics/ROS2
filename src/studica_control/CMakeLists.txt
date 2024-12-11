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
find_package(rosidl_typesupport_c REQUIRED)

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
  include/studica_control/encoder_component.h
  include/studica_control/servo_component.h
  include/studica_control/titan_component.h
  include/studica_control/ultrasonic_component.h
  include/studica_control/sharp_component.h
  include/studica_control/cobra_component.h
  include/studica_control/imu_component.h
  include/studica_control/dio_component.h
)

set(dependencies "ament_index_cpp" "sensor_msgs" "rclcpp_components" "rclcpp" "rclcpp_action" "std_msgs" "std_srvs")

# Define components as shared libraries
add_library(encoder_component SHARED src/components/encoder_component.cpp)
ament_target_dependencies(encoder_component ${dependencies})
rclcpp_components_register_nodes(encoder_component "studica_control::encoder")

add_library(servo_component SHARED src/components/servo_component.cpp)
ament_target_dependencies(servo_component ${dependencies})
rclcpp_components_register_nodes(servo_component "studica_control::Servo")

add_library(titan_component SHARED src/components/titan_component.cpp)
ament_target_dependencies(titan_component ${dependencies})
rclcpp_components_register_nodes(titan_component "studica_control::Titan")

add_library(ultrasonic_component SHARED src/components/ultrasonic_component.cpp)
ament_target_dependencies(ultrasonic_component ${dependencies})
rclcpp_components_register_nodes(ultrasonic_component "studica_control::ultrasonic")

add_library(sharp_component SHARED src/components/sharp_component.cpp)
ament_target_dependencies(sharp_component ${dependencies})
rclcpp_components_register_nodes(sharp_component "studica_control::sharp")

add_library(cobra_component SHARED src/components/cobra_component.cpp)
ament_target_dependencies(cobra_component ${dependencies})
rclcpp_components_register_nodes(cobra_component "studica_control::cobra")

add_library(imu_component SHARED src/components/imu_component.cpp)
ament_target_dependencies(imu_component ${dependencies})
rclcpp_components_register_nodes(imu_component "studica_control::imu")

add_library(dio_component SHARED src/components/dio_component.cpp)
ament_target_dependencies(dio_component ${dependencies})
rclcpp_components_register_nodes(dio_component "studica_control::DIO")

# Install components
install(TARGETS 
  encoder_component
  servo_component
  ultrasonic_component
  sharp_component
  cobra_component
  titan_component
  imu_component
  dio_component
  # ARCHIVE DESTINATION lib
  # LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin

  ARCHIVE DESTINATION /usr/local/lib/studica_control
  LIBRARY DESTINATION /usr/local/lib/studica_control
  RUNTIME DESTINATION /usr/local/bin/studica_control
)

# Executables
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(manual_composition src/manual_composition.cpp)
target_link_libraries(manual_composition 
  encoder_component
  servo_component
  ultrasonic_component
  sharp_component
  cobra_component
  titan_component
  imu_component
  dio_component
  /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so
  /usr/local/lib/studica_drivers/libstudica_drivers.so
)
ament_target_dependencies(manual_composition ${dependencies})
target_link_libraries(manual_composition "${cpp_typesupport_target}")

# Install executable
install(TARGETS
  manual_composition
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(manual_composition2 src/manual_composition2.cpp)
target_link_libraries(manual_composition2 
  encoder_component
  servo_component
  ultrasonic_component
  sharp_component
  cobra_component
  titan_component
  imu_component
  dio_component
  /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so
  /usr/local/lib/studica_drivers/libstudica_drivers.so
)
ament_target_dependencies(manual_composition2 ${dependencies})
target_link_libraries(manual_composition2 "${cpp_typesupport_target}")

# Install executable
install(TARGETS
  manual_composition2
  DESTINATION lib/${PROJECT_NAME}
)

# # Component test executable
# add_executable(comp_test src/comp_test.cpp)
# target_link_libraries(comp_test 
#   encoder_component
#   servo_component
#   sharp_component
#   cobra_component
#   titan_component
#   dio_component
#   /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so
#   /usr/local/lib/studica_drivers/libstudica_drivers.so
# )
# ament_target_dependencies(comp_test ${dependencies})
# target_link_libraries(comp_test "${cpp_typesupport_target}")


# install(TARGETS
#   comp_test
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install include files and generated headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/
  DESTINATION include/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION /usr/local/include/studica_control
)


# Export package information
ament_export_include_directories(include)
ament_export_libraries(
  encoder_component 
  servo_component 
  ultrasonic_component 
  sharp_component 
  cobra_component 
  titan_component
  dio_component
)
ament_export_dependencies(${dependencies})

# Set RPATH
set(CMAKE_INSTALL_RPATH "/usr/local/lib/studica_drivers")

ament_package()

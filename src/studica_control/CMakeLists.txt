cmake_minimum_required(VERSION 3.8)
project(studica_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} # <- SRV files (srv, msg, action)
  "msg/DutyCycleEncoderMsg.msg"
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
  include/studica_control/cobra_component.h
  include/studica_control/device.h
  include/studica_control/dc_encoder_component.h
  include/studica_control/diff_drive_component.h
  include/studica_control/diff_drive_odometry.h
  include/studica_control/dio_component.h
  include/studica_control/encoder_component.h
  include/studica_control/imu_component.h
  include/studica_control/mecanum_drive_component.h
  include/studica_control/servo_component.h
  include/studica_control/sharp_component.h
  include/studica_control/titan_component.h
  include/studica_control/ultrasonic_component.h
)

set(dependencies 
  "ament_index_cpp" 
  "nav_msgs"
  "rclcpp" 
  "rclcpp_action"
  "rclcpp_components" 
  "sensor_msgs"  
  "std_msgs" 
  "std_srvs" 
  "tf2"
  "tf2_ros"
)

add_library(cobra_component SHARED src/components/cobra_component.cpp)
ament_target_dependencies(cobra_component ${dependencies})
rclcpp_components_register_nodes(cobra_component "studica_control::cobra")

add_library(dc_encoder_component SHARED src/components/dc_encoder_component.cpp)
ament_target_dependencies(dc_encoder_component ${dependencies})
rclcpp_components_register_nodes(dc_encoder_component "studica_control::DutyCycleEncoder")

add_library(diff_drive_component SHARED src/components/diff_drive_component.cpp)
ament_target_dependencies(diff_drive_component ${dependencies})
rclcpp_components_register_nodes(diff_drive_component "studica_control::DiffDrive")

add_library(diff_drive_odometry SHARED src/components/diff_drive_odometry.cpp)
ament_target_dependencies(diff_drive_odometry ${dependencies})
rclcpp_components_register_nodes(diff_drive_odometry "studica_control::DiffOdometry")

add_library(dio_component SHARED src/components/dio_component.cpp)
ament_target_dependencies(dio_component ${dependencies})
rclcpp_components_register_nodes(dio_component "studica_control::DIO")

add_library(encoder_component SHARED src/components/encoder_component.cpp)
ament_target_dependencies(encoder_component ${dependencies})
rclcpp_components_register_nodes(encoder_component "studica_control::encoder")

add_library(imu_component SHARED src/components/imu_component.cpp)
ament_target_dependencies(imu_component ${dependencies})
rclcpp_components_register_nodes(imu_component "studica_control::imu")

add_library(mecanum_drive_component SHARED src/components/mecanum_drive_component.cpp)
ament_target_dependencies(mecanum_drive_component ${dependencies})
rclcpp_components_register_nodes(mecanum_drive_component "studica_control::MecanumDrive")

add_library(mecanum_drive_odometry SHARED src/components/mecanum_drive_odometry.cpp)
ament_target_dependencies(mecanum_drive_odometry ${dependencies})

add_library(servo_component SHARED src/components/servo_component.cpp)
ament_target_dependencies(servo_component ${dependencies})
rclcpp_components_register_nodes(servo_component "studica_control::Servo")

add_library(sharp_component SHARED src/components/sharp_component.cpp)
ament_target_dependencies(sharp_component ${dependencies})
rclcpp_components_register_nodes(sharp_component "studica_control::sharp")

add_library(titan_component SHARED src/components/titan_component.cpp)
ament_target_dependencies(titan_component ${dependencies})
rclcpp_components_register_nodes(sharp_component "studica_control::Titan")

add_library(ultrasonic_component SHARED src/components/ultrasonic_component.cpp)
ament_target_dependencies(ultrasonic_component ${dependencies})
rclcpp_components_register_nodes(ultrasonic_component "studica_control::Ultrasonic")

target_link_libraries(diff_drive_component diff_drive_odometry)
target_link_libraries(mecanum_drive_component mecanum_drive_odometry)

install(TARGETS 
  cobra_component
  dc_encoder_component
  diff_drive_component
  diff_drive_odometry
  dio_component
  encoder_component
  imu_component
  mecanum_drive_component
  mecanum_drive_odometry
  servo_component
  sharp_component
  titan_component
  ultrasonic_component  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(manual_composition src/manual_composition.cpp)
target_link_libraries(manual_composition 
  cobra_component
  dc_encoder_component
  diff_drive_component
  diff_drive_odometry
  dio_component
  encoder_component
  imu_component
  mecanum_drive_component
  mecanum_drive_odometry
  servo_component
  sharp_component
  titan_component
  ultrasonic_component
  /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so
  /usr/local/lib/studica_drivers/libstudica_drivers.so
)
ament_target_dependencies(manual_composition ${dependencies})
target_link_libraries(manual_composition "${cpp_typesupport_target}")

install(TARGETS
  manual_composition
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION /usr/local/include/studica_control
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
  cobra_component
  dc_encoder_component
  diff_drive_component
  diff_drive_odometry
  dio_component
  encoder_component
  imu_component
  mecanum_drive_component
  servo_component
  sharp_component
  titan_component
  ultrasonic_component
)
ament_export_dependencies(${dependencies})

set(CMAKE_INSTALL_RPATH "/usr/local/lib/studica_drivers")

ament_package()

cmake_minimum_required(VERSION 3.8)
project(studica_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} # <- SRV files (srv, msg, action)
  "msg/DutyCycleEncoderMsg.msg"
  "msg/EncoderMsg.msg"
  "msg/InitializeParams.msg"
  "srv/ControlImu.srv"
  "srv/SetData.srv"
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

include_directories( # <- #include <"">
  include 
  /usr/local/include/vmxpi 
  /usr/local/include/studica_drivers
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
 
set(studica_control_HDRS
  include/studica_control/cobra_component.hpp
  include/studica_control/dc_encoder_component.hpp
  include/studica_control/dio_component.hpp
  include/studica_control/encoder_component.hpp
  include/studica_control/gamepad_component.hpp
  include/studica_control/imu_component.hpp
  include/studica_control/servo_component.hpp
  include/studica_control/sharp_component.hpp
  include/studica_control/titan_component.hpp
  include/studica_control/ultrasonic_component.hpp
)

set(dependencies
  "ament_index_cpp"
  "geometry_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
)

add_library(cobra_component SHARED src/components/cobra_component.cpp)
ament_target_dependencies(cobra_component ${dependencies})
target_link_libraries(cobra_component "${cpp_typesupport_target}")
rclcpp_components_register_nodes(cobra_component "studica_control::cobra")

add_library(dc_encoder_component SHARED src/components/dc_encoder_component.cpp)
ament_target_dependencies(dc_encoder_component ${dependencies})
target_link_libraries(dc_encoder_component "${cpp_typesupport_target}")
rclcpp_components_register_nodes(dc_encoder_component "studica_control::DutyCycleEncoder")

add_library(dio_component SHARED src/components/dio_component.cpp)
ament_target_dependencies(dio_component ${dependencies})
target_link_libraries(dio_component "${cpp_typesupport_target}")
rclcpp_components_register_nodes(dio_component "studica_control::DIO")

add_library(encoder_component SHARED src/components/encoder_component.cpp)
ament_target_dependencies(encoder_component ${dependencies})
target_link_libraries(encoder_component "${cpp_typesupport_target}")
rclcpp_components_register_nodes(encoder_component "studica_control::encoder")

add_library(gamepad_component SHARED src/components/gamepad_component.cpp)
ament_target_dependencies(gamepad_component ${dependencies})
target_link_libraries(gamepad_component "${cpp_typesupport_target}")
rclcpp_components_register_nodes(gamepad_component "studica_control::GamepadController")

add_library(imu_component SHARED src/components/imu_component.cpp)
ament_target_dependencies(imu_component ${dependencies})
target_link_libraries(imu_component "${cpp_typesupport_target}")
rclcpp_components_register_nodes(imu_component "studica_control::imu")

add_library(servo_component SHARED src/components/servo_component.cpp)
ament_target_dependencies(servo_component ${dependencies})
target_link_libraries(servo_component "${cpp_typesupport_target}")
rclcpp_components_register_nodes(servo_component "studica_control::Servo")

add_library(sharp_component SHARED src/components/sharp_component.cpp)
ament_target_dependencies(sharp_component ${dependencies})
target_link_libraries(sharp_component "${cpp_typesupport_target}")
rclcpp_components_register_nodes(sharp_component "studica_control::sharp")

add_library(titan_component SHARED src/components/titan_component.cpp)
ament_target_dependencies(titan_component ${dependencies})
target_link_libraries(titan_component "${cpp_typesupport_target}")
rclcpp_components_register_nodes(titan_component "studica_control::Titan")

add_library(ultrasonic_component SHARED src/components/ultrasonic_component.cpp)
ament_target_dependencies(ultrasonic_component ${dependencies})
target_link_libraries(ultrasonic_component "${cpp_typesupport_target}")
rclcpp_components_register_nodes(ultrasonic_component "studica_control::Ultrasonic")

install(TARGETS
  cobra_component
  dc_encoder_component
  dio_component
  encoder_component
  gamepad_component
  imu_component
  servo_component
  sharp_component
  titan_component
  ultrasonic_component  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  RUNTIME DESTINATION bin
)

add_executable(manual_composition src/manual_composition.cpp)
target_link_libraries(manual_composition
  cobra_component
  dc_encoder_component
  dio_component
  encoder_component
  gamepad_component
  imu_component
  servo_component
  sharp_component
  titan_component
  ultrasonic_component
  /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so
  /usr/local/lib/studica_drivers/libstudica_drivers.so
)
ament_target_dependencies(manual_composition ${dependencies})
target_link_libraries(manual_composition "${cpp_typesupport_target}")
# $ORIGIN/.. resolves to install/studica_control/lib/ at runtime, where the
# component .so files live. The vmxpi and studica_drivers paths are for the
# hardware driver libs installed system-wide.
set_target_properties(manual_composition PROPERTIES
  INSTALL_RPATH "$ORIGIN/..;/usr/local/lib/vmxpi;/usr/local/lib/studica_drivers"
)

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

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
  cobra_component
  dc_encoder_component
  dio_component
  encoder_component
  gamepad_component
  imu_component
  servo_component
  sharp_component
  titan_component
  ultrasonic_component
)
ament_export_dependencies(${dependencies})

# ---- C++ examples ----
set(cpp_example_targets
  titan_example
  imu_example
  servo_example
  ultrasonic_example
  sharp_example
  encoder_example
  dc_encoder_example
  dio_example
  cobra_example
  gamepad_example
)

foreach(target ${cpp_example_targets})
  add_executable(${target} src/components/examples/cpp/${target}.cpp)
  ament_target_dependencies(${target} rclcpp std_msgs sensor_msgs geometry_msgs)
  target_link_libraries(${target} "${cpp_typesupport_target}")
endforeach()

install(TARGETS ${cpp_example_targets}
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Python examples ----
set(python_example_scripts
  src/components/examples/python/titan_example.py
  src/components/examples/python/imu_example.py
  src/components/examples/python/servo_example.py
  src/components/examples/python/ultrasonic_example.py
  src/components/examples/python/sharp_example.py
  src/components/examples/python/encoder_example.py
  src/components/examples/python/dc_encoder_example.py
  src/components/examples/python/dio_example.py
  src/components/examples/python/cobra_example.py
  src/components/examples/python/gamepad_example.py
)

install(PROGRAMS ${python_example_scripts}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

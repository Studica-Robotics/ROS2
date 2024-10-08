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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include /usr/local/include/vmxpi) # <- #include <"">

rosidl_generate_interfaces(${PROJECT_NAME} # <- SRV files
  # "action/RotateAbsolute.action"
  "msg/InitializeParams.msg"
  "srv/ControlImu.srv"
  "srv/SetData.srv"
  )

set(dependencies "ament_index_cpp" "sensor_msgs" "rclcpp" "rclcpp_action" "std_msgs" "std_srvs")


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()


# ----------------------------------------------------------------------------------
set(studica_control_SRCS # <- source files
  src/studica_control_node.cpp
  src/imu_driver_node.cpp
  src/dynamic_publisher.cpp
  src/ultrasonic.cpp
  src/sharp_sensor_node.cpp
  src/DIOPin.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(studica_control_node ${studica_control_SRCS} include/studica_control include/studica_control/imu_driver_node.h include/studica_control/ultrasonic.h)
target_link_libraries(studica_control_node /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so)
ament_target_dependencies(studica_control_node ${dependencies})
target_link_libraries(studica_control_node "${cpp_typesupport_target}")

add_executable(tester src/test.cpp)
ament_target_dependencies(tester ${dependencies})
target_link_libraries(tester "${cpp_typesupport_target}")


# add_executable(dio src/dio.cpp)
# ament_target_dependencies(dio ${dependencies})
# target_link_libraries(dio "${cpp_typesupport_target}")


install(TARGETS studica_control_node tester  # <- Executable targets
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

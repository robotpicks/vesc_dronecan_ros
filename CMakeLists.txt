cmake_minimum_required(VERSION 3.20)
project(rp1_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Vendored libcanard: the same UAVCAN v0 codec used by the firmware
# (/home/user/dev/bldc/libcanard), copied here rather than depended on since it's not a
# packaged library. CANARD_ENABLE_CANFD=1 is required on this 64-bit PC target -- see the
# rp1-vendoring comments in vendor/libcanard/canard.h/canard_internals.h for why (it fixes a
# pointer-size-dependent memory layout assumption that only holds on 32-bit embedded targets;
# it does NOT enable actual CAN-FD wire framing here, all canardBroadcast() calls pass
# canfd=false).
add_library(rp1_canard STATIC
  vendor/libcanard/canard.c
  vendor/libcanard/dsdl/uavcan/equipment/actuator/actuator_Command.c
  vendor/libcanard/dsdl/uavcan/equipment/actuator/actuator_ArrayCommand.c
  vendor/libcanard/dsdl/uavcan/equipment/actuator/actuator_Status.c
  vendor/libcanard/dsdl/uavcan/equipment/esc/esc_RPMCommand.c
  vendor/libcanard/dsdl/uavcan/equipment/esc/esc_Status.c
)
target_include_directories(rp1_canard PUBLIC
  vendor/libcanard
  vendor/libcanard/dsdl
)
target_compile_definitions(rp1_canard PUBLIC CANARD_ENABLE_CANFD=1)
set_target_properties(rp1_canard PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(rp1_hardware_interface SHARED
  src/rp1_hardware.cpp
)
target_compile_features(rp1_hardware_interface PUBLIC cxx_std_17)
target_include_directories(rp1_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rp1_hardware_interface PRIVATE rp1_canard)
# Modern namespaced imported targets rather than ament_target_dependencies(): that macro is gone
# in current ament_cmake (no longer shipped by ament_cmake_target_dependencies), and the
# ::-targets carry their own include dirs, so this is the portable spelling.
target_link_libraries(rp1_hardware_interface PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface rp1_hardware_interface.xml)

install(
  TARGETS rp1_hardware_interface
  EXPORT export_rp1_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(FILES rp1_hardware_interface.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(rp1_hardware_interface)
ament_export_targets(export_rp1_hardware_interface)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()

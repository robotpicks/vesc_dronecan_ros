cmake_minimum_required(VERSION 3.20)
project(vesc_dronecan_driver)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Vendored libcanard: the same UAVCAN v0 codec the VESC firmware itself uses (bldc/libcanard),
# copied here rather than depended on since it is not a packaged library. Keeping the codec
# identical on both ends is the point -- encode/decode bugs cancel out rather than compound.
# CANARD_ENABLE_CANFD=1 is required on a 64-bit PC target -- see the vendoring
# comments in vendor/libcanard/canard.h/canard_internals.h for why (it fixes a
# pointer-size-dependent memory layout assumption that only holds on 32-bit embedded targets;
# it does NOT enable actual CAN-FD wire framing here, all canardBroadcast() calls pass
# canfd=false).
add_library(vesc_canard STATIC
  vendor/libcanard/canard.c
  vendor/libcanard/dsdl/uavcan/equipment/actuator/actuator_Command.c
  vendor/libcanard/dsdl/uavcan/equipment/actuator/actuator_ArrayCommand.c
  vendor/libcanard/dsdl/uavcan/equipment/actuator/actuator_Status.c
  vendor/libcanard/dsdl/uavcan/equipment/esc/esc_RPMCommand.c
  vendor/libcanard/dsdl/uavcan/equipment/esc/esc_Status.c
)
target_include_directories(vesc_canard PUBLIC
  vendor/libcanard
  vendor/libcanard/dsdl
)
target_compile_definitions(vesc_canard PUBLIC CANARD_ENABLE_CANFD=1)
set_target_properties(vesc_canard PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(vesc_dronecan_driver SHARED
  src/vesc_dronecan_system.cpp
)
target_compile_features(vesc_dronecan_driver PUBLIC cxx_std_17)
target_include_directories(vesc_dronecan_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vesc_dronecan_driver PRIVATE vesc_canard)
# Modern namespaced imported targets rather than ament_target_dependencies(): that macro is gone
# in current ament_cmake (no longer shipped by ament_cmake_target_dependencies), and the
# ::-targets carry their own include dirs, so this is the portable spelling.
target_link_libraries(vesc_dronecan_driver PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface vesc_dronecan_driver.xml)

install(
  TARGETS vesc_dronecan_driver
  EXPORT export_vesc_dronecan_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
# urdf/ holds only the worked example. Robot descriptions belong in the robot's own
# <robot>_description package -- nothing here should be depended on by a launch file.
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(FILES vesc_dronecan_driver.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(vesc_dronecan_driver)
ament_export_targets(export_vesc_dronecan_driver)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()

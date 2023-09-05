cmake_minimum_required(VERSION 3.8)
project(fws_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  fws_controller
  SHARED
  src/fws_controller.cpp
)

target_compile_features(fws_controller PUBLIC cxx_std_17)
target_include_directories(fws_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/fws_controller>
)

ament_target_dependencies(
  fws_controller
  ${CONTROLLER_INCLUDE_DEPENDS}
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(controller_interface fws_controller_plugin.xml)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(
  TARGETS fws_controller
  EXPORT fws_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(fws_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS})
ament_package()

cmake_minimum_required(VERSION 3.8)
project(dua_rviz_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(dua_interfaces REQUIRED)
find_package(dua_qos_cpp REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/dua_rviz_plugins/start_stop_panel.hpp
  include/dua_rviz_plugins/text_sub_panel.hpp
  include/dua_rviz_plugins/text_pub_panel.hpp
  include/dua_rviz_plugins/visual_targets_display.hpp
)

FILE(GLOB_RECURSE PLUGIN_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
add_library(dua_rviz_plugins SHARED
  ${PLUGIN_SOURCES}
  ${MOC_FILES}
)
target_include_directories(dua_rviz_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dua_rviz_plugins
  ${Qt5Widgets_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Gui_LIBRARIES}
  yaml-cpp
)
ament_target_dependencies(dua_rviz_plugins
  dua_interfaces
  dua_qos_cpp
  interactive_markers
  pluginlib
  Qt5
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_rendering
  sensor_msgs
  std_msgs
  vision_msgs
  visualization_msgs
)

install(TARGETS dua_rviz_plugins
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)

install(FILES dua_rviz_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

file(GLOB ICON_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} icons/*)
install(FILES
  ${ICON_FILES}
  DESTINATION share/${PROJECT_NAME}/icons/classes
)

ament_export_include_directories(include)
pluginlib_export_plugin_description_file(rviz_common dua_rviz_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

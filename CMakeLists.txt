cmake_minimum_required(VERSION 3.10)
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

find_package(ament_cmake REQUIRED)
find_package(dua_cv_bridge REQUIRED)
find_package(dua_mission_interfaces REQUIRED)
find_package(dua_qos_cpp REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(OpenCV 4.11.0 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui Network)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(CURL REQUIRED)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOMOC ON)

set(PANEL_SRCS
  src/panels/start_stop/start_stop_panel.cpp
  src/panels/text_pub/text_pub_panel.cpp
  src/panels/text_sub/text_sub_panel.cpp
)

set(DISPLAY_SRCS
  src/displays/attitude_indicator/attitude_indicator_display.cpp
  src/displays/attitude_indicator/attitude_indicator_widget.cpp
  src/displays/geo_view/geo_view_display.cpp
  src/displays/visual_targets/visual_targets_display.cpp
)

set(MOC_HEADERS
  include/dua_rviz_plugins/panels/start_stop/start_stop_panel.hpp
  include/dua_rviz_plugins/panels/text_pub/text_pub_panel.hpp
  include/dua_rviz_plugins/panels/text_sub/text_sub_panel.hpp
  include/dua_rviz_plugins/displays/attitude_indicator/attitude_indicator_display.hpp
  include/dua_rviz_plugins/displays/attitude_indicator/attitude_indicator_widget.hpp
  include/dua_rviz_plugins/displays/geo_view/geo_view_display.hpp
  include/dua_rviz_plugins/displays/visual_targets/visual_targets_display.hpp
)

add_library(dua_rviz_plugins SHARED
  ${PANEL_SRCS}
  ${DISPLAY_SRCS}
  ${MOC_HEADERS}
)
target_include_directories(dua_rviz_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(dua_rviz_plugins
  ${OpenCV_LIBS}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::Network
  yaml-cpp
  SQLite::SQLite3
  CURL::libcurl
)

ament_target_dependencies(dua_rviz_plugins
  dua_cv_bridge
  dua_mission_interfaces
  dua_qos_cpp
  interactive_markers
  pluginlib
  Qt5
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_rendering
  sensor_msgs
  visualization_msgs
  tf2
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

install(DIRECTORY icons/
        DESTINATION share/${PROJECT_NAME}/icons
)

install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY dae/
        DESTINATION share/${PROJECT_NAME}/dae
)

ament_export_include_directories(include)
pluginlib_export_plugin_description_file(rviz_common dua_rviz_plugins.xml)

ament_package()

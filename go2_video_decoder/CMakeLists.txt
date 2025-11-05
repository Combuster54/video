project(go2_video_decoder)
cmake_minimum_required(VERSION 3.8)

# ROS2 Foxy setup
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(unitree_go REQUIRED)

# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
)

# Source files
set(SOURCES
  src/go2_video_decoder.cpp
)

# Header files
set(HEADERS
  include/go2_video_decoder/go2_video_decoder.hpp
)

# Create library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Create executable node
add_executable(go2_video_decoder_node src/go2_video_decoder_node.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
)

# ROS2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  cv_bridge
)

# Link node to library
target_link_libraries(go2_video_decoder_node
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
)

# ROS2 dependencies for node
ament_target_dependencies(go2_video_decoder_node
  rclcpp
  sensor_msgs
  cv_bridge
  unitree_go
)

# Install rules
install(TARGETS ${PROJECT_NAME} go2_video_decoder_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# ROS2 export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
  unitree_go
)

ament_package()


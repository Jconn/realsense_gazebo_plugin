cmake_minimum_required(VERSION 3.5)
project(realsense_gazebo_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()


find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)


link_directories(${gazebo_dev_LIBRARY_DIRS})


# gazebo_ros_diff_drive
add_library(realsense_gazebo_plugin SHARED
    src/RealSensePlugin.cpp
    src/gazebo_ros_realsense.cpp
)
ament_target_dependencies(realsense_gazebo_plugin
  "camera_info_manager"
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)


ament_export_libraries(realsense_gazebo_plugin)

ament_export_include_directories(include)

ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_ros)

#if(BUILD_TESTING)
#  add_subdirectory(test)
# find_package(ament_lint_auto REQUIRED)
# ament_lint_auto_find_test_dependencies()
#endif()

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    realsense_gazebo_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

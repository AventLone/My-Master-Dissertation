cmake_minimum_required(VERSION 3.22)
project(webots_simulation LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# --------------------------------------------------------------------------------
# Find dependent ROS2 packages
# --------------------------------------------------------------------------------
set(DEPENDENT_PACKAGIES
    rclcpp
    sensor_msgs
    pluginlib
    webots_ros2_driver
)
find_package(ament_cmake REQUIRED)

foreach(PACKAGE IN ITEMS ${DEPENDENT_PACKAGIES})
    find_package(${PACKAGE} REQUIRED)
endforeach()

# Besides the package specific dependencies we also need the `pluginlib` and
# `webots_ros2_driver`

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver track_cart_driver.xml)

# MyRobotDriver library
add_library(${PROJECT_NAME} SHARED src/MyRobotDriver.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENT_PACKAGIES})
install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install additional directories.
install(DIRECTORY launch worlds urdf
    DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()

cmake_minimum_required(VERSION 3.8)
project(webots_simulation)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

set(DEPENDENT_PACKAGIES
    rclcpp
    sensor_msgs
    pluginlib
    webots_ros2_driver
)

foreach(PACKAGE IN ITEMS ${DEPENDENT_PACKAGIES})
    find_package(${PACKAGE} REQUIRED)
endforeach()

# Besides the package specific dependencies we also need the `pluginlib` and
# `webots_ros2_driver`

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver track_cart_driver.xml)

# RobotDriver library
add_library(${PROJECT_NAME} SHARED src/RobotDriver.cpp)
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

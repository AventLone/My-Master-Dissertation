cmake_minimum_required(VERSION 3.8)
project(track_cart_drive_sim LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

set(PACKEGE_DEPENDS
    rclcpp
    sensor_msgs
    pluginlib
    webots_ros2_driver
)

foreach(Dependency IN ITEMS ${PACKEGE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

# Besides the package specific dependencies we also need the `pluginlib` and
# `webots_ros2_driver`

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver track_cart_driver.xml)

# Obstacle avoider
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# add_executable(obstacle_avoider
# src/ObstacleAvoider.cpp
# )
# ament_target_dependencies(obstacle_avoider
# rclcpp
# geometry_msgs
# sensor_msgs
# )
# install(TARGETS obstacle_avoider
# DESTINATION lib/${PROJECT_NAME}
# )
# install(DIRECTORY include/
# DESTINATION include
# )

# MyRobotDriver library
add_library(${PROJECT_NAME} SHARED src/MyRobotDriver.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(${PROJECT_NAME}
    ${PACKEGE_DEPENDS}
)
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

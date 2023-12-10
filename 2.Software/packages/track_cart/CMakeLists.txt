cmake_minimum_required(VERSION 3.8)
project(track_cart)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find ROS2 dependencies
set(PACKAGE_DEPENDS
    rclcpp
    sensor_msgs
    nav_msgs
    cv_bridge
    tf2
    # tf2_geometry_msgs
    tf2_ros
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${PACKAGE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# file(GLOB SOURCE_DIR_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# add_executable(rgbd ${SOURCE_DIR_FILES})

# target_compile_features(rgbd PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
# ament_target_dependencies(rgbd ${PACKAGE_DEPENDS})

# install(TARGETS rgbd
# DESTINATION lib/${PROJECT_NAME}
# )
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

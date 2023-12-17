cmake_minimum_required(VERSION 3.22)
project(aloam LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------------
# Find ROS2 dependencies
# --------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)

set(DEPENDENT_PACKAGIES
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    pcl_conversions
)

foreach(PACKAGE IN ITEMS ${DEPENDENT_PACKAGIES})
    find_package(${PACKAGE} REQUIRED)
endforeach()

# --------------------------------------------------------------------------------
# Find dependencies
# --------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

# Find all source files
# file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
# file(GLOB SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
set(NODE_NAME aloam_system)
add_executable(${NODE_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SystemNode.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Localizing.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Mapping.cpp
)

# Specify a C++ standard
target_compile_features(${NODE_NAME} PUBLIC c_std_99 cxx_std_17)

# Specify the ROS2 package dependencies, and link libraries
ament_target_dependencies(${NODE_NAME} ${DEPENDENT_PACKAGIES})
target_link_libraries(${NODE_NAME}
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)
install(TARGETS ${NODE_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

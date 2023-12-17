cmake_minimum_required(VERSION 3.22)
project(faster_lio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Add definitions
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# options
option(WITH_IVOX_NODE_TYPE_PHC "Use PHC instead of default ivox node" OFF)

if(WITH_IVOX_NODE_TYPE_PHC)
    message("USING_IVOX_NODE_TYPE_PHC")
    add_definitions(-DIVOX_NODE_TYPE_PHC)
else()
    message("USING_IVOX_NODE_TYPE_DEFAULT")
endif()

# --------------------------------------------------------------------------------
# Find dependent ROS2 packages
# --------------------------------------------------------------------------------
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
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach(PACKAGE IN ITEMS ${DEPENDENT_PACKAGIES})
    find_package(${PACKAGE} REQUIRED)
endforeach()

# --------------------------------------------------------------------------------
# Find dependencies
# --------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
set(DEPENDENT_LIBS
    ${PCL_LIBRARIES}
)

# Generate the customized msg interface
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Pose6D.msg"
)

# ament_export_dependencies(rosidl_default_runtime)

# Find all source files
file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# Specify a name of the node(executable)
set(NODE_NAME faster_lio_system)
add_executable(${NODE_NAME} ${SOURCE_FILES})
target_include_directories(${NODE_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

# Specify the C/C++ standard
target_compile_features(${NODE_NAME} PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_link_libraries(${NODE_NAME} ${DEPENDENT_LIBS})
ament_target_dependencies(${NODE_NAME} ${DEPENDENT_PACKAGIES})
rosidl_target_interfaces(${NODE_NAME}
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install the executable and directories in specified folders
install(TARGETS ${NODE_NAME}
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)
ament_package()

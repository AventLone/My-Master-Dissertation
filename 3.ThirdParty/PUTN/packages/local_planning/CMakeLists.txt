cmake_minimum_required(VERSION 3.22)
project(local_planning LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# --------------------------------------------------------------------------------
# Find dependent ROS2 packages
# --------------------------------------------------------------------------------
set(DEPENDENT_PACKAGIES
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2_ros
)
find_package(ament_cmake REQUIRED)

foreach(PACKAGE IN ITEMS ${DEPENDENT_PACKAGIES})
    find_package(${PACKAGE} REQUIRED)
endforeach()

# --------------------------------------------------------------------------------
# Find dependencies
# --------------------------------------------------------------------------------
find_package(casadi REQUIRED)
find_package(Eigen3 REQUIRED)
set(DEPENDENT_LIBS
    casadi
)

# Find all source files
file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# Specify a name of the node(executable)
set(NODE_NAME local_planner)
add_executable(${NODE_NAME} ${SOURCE_FILES})
target_include_directories(${NODE_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CASADI_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

# Specify a C++ standard
target_compile_features(${NODE_NAME} PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_link_libraries(${NODE_NAME} ${DEPENDENT_LIBS})
ament_target_dependencies(${NODE_NAME} ${DEPENDENT_PACKAGIES})

# Install the executable and directories in specified folders
install(TARGETS ${NODE_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY launch config model
# DESTINATION share/${PROJECT_NAME}
# )
ament_package()

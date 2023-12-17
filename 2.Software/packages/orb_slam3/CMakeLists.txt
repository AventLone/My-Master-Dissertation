cmake_minimum_required(VERSION 3.8)
project(orb_slam3 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# --------------------------------------------------------------------------------
# Find ROS2 dependencies
# --------------------------------------------------------------------------------
set(DEPENDENT_PACKAGIES
    rclcpp
    sensor_msgs
    cv_bridge
    tf2
    tf2_eigen
    tf2_ros
    OpenCV
    pcl_conversions
    message_filters
    octomap
    octomap_msgs
    nav_msgs
)

find_package(ament_cmake REQUIRED)

foreach(PACKAGE IN ITEMS ${DEPENDENT_PACKAGIES})
    find_package(${PACKAGE} REQUIRED)
endforeach()

# --------------------------------------------------------------------------------
# Find dependencies
# --------------------------------------------------------------------------------
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(CUDA REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/api
    ${CMAKE_CURRENT_SOURCE_DIR}/api/include
    ${CMAKE_CURRENT_SOURCE_DIR}/api/include/CameraModels
    ${CMAKE_CURRENT_SOURCE_DIR}/api/include/PointCloud&Seg
    ${CMAKE_CURRENT_SOURCE_DIR}/api/Thirdparty/Sophus
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Pangolin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    /home/avent/.CppLib/TensorRT-8.5.3.1/include
)

set(DEPENDENT_LIBS
    ${Pangolin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/api/lib/libDBoW2.so
    ${CMAKE_CURRENT_SOURCE_DIR}/api/lib/libg2o.so
    ${CMAKE_CURRENT_SOURCE_DIR}/api/lib/libORB_SLAM3_Dense.so
    -lboost_system
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Find all source files
file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# Specify a name of the node(executable)
set(NODE_NAME orb_slam3)
add_executable(${NODE_NAME} ${SOURCE_FILES})

target_compile_features(${NODE_NAME} PUBLIC c_std_99 cxx_std_17) # Specify a C++ standard: Require C99 and C++17
target_link_libraries(${NODE_NAME} ${DEPENDENT_LIBS})
ament_target_dependencies(${NODE_NAME} ${DEPENDENT_PACKAGIES})

# Install the executable and directories in specified folders
install(TARGETS ${NODE_NAME}
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config model
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

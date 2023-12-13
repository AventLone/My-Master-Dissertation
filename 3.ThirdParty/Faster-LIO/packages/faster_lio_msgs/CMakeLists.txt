cmake_minimum_required(VERSION 3.22)
project(faster_lio_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------------
# Find dependent ROS2 packages
# --------------------------------------------------------------------------------
set(DEPENDENT_PACKAGIES
    rosidl_default_generators
)
find_package(ament_cmake REQUIRED)

foreach(PACKAGE IN ITEMS ${DEPENDENT_PACKAGIES})
    find_package(${PACKAGE} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/Pose6D.msg
)

ament_package()

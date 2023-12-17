cmake_minimum_required(VERSION 3.8)
project(ORB_SLAM3_Dense)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set(CMAKE_CXX_STANDARD 17)

# Find dependencies the project need.
set(DEPENDENCIES
    OpenCV
    Eigen3
    Pangolin
    PCL
    CUDA
)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
    find_package(${Dependency} REQUIRED)
endforeach()

# add TensorRT8
file(GLOB TENSORRT_LIBS /home/avent/.CppLib/TensorRT-8.5.3.1/lib/*.so)

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/CameraModels
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PointCloud&Seg
    ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Sophus
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    /home/avent/.CppLib/TensorRT-8.5.3.1/include
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    -lboost_serialization
    -lcrypto
    ${CUDA_LIBRARIES}
    ${TENSORRT_LIBS}
)
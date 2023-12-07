cmake_minimum_required(VERSION 2.8.3)
project(go1_cam)

find_package(OpenCV 4 REQUIRED)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp image_transport camera_info_manager cv_bridge
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS
        roscpp image_transport camera_info_manager cv_bridge
)

include_directories(
        ./include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    link_directories(${PROJECT_SOURCE_DIR}/lib/amd64)
else()
    link_directories(${PROJECT_SOURCE_DIR}/lib/arm64)
endif ()

set(SDKLIBS unitree_camera tstc_V4L2_xu_camera udev systemlog ${OpenCV_LIBS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

add_executable(go1_cam src/go1_cam.cpp)
target_link_libraries(go1_cam ${SDKLIBS} ${catkin_LIBRARIES})

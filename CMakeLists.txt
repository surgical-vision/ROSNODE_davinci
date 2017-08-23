cmake_minimum_required(VERSION 2.8)
project(ROSNODE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, defaulting to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()


find_package(PAWSS REQUIRED)
if(PAWSS_FOUND)
    message(STATUS "PAWSS Found and Enabled.")
endif()

set(OpenCV_DIR "/usr/local/opt/opencv3/share/OpenCV")
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    set(HAVE_OPENCV 1)
    list(APPEND INTERNAL_INC ${OpenCV_INCLUDE_DIRS})
    list(APPEND LINK_LIBS ${OpenCV_LIBS})
    message(STATUS "OpenCV Found and Enabled.")
endif()



include_directories(${PAWSS_INCLUDE_DIRS})

add_executable(ROSNODE object_tracking_node.cpp)
target_link_libraries(ROSNODE ${PAWSS_LIBRARIES})

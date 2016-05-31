cmake_minimum_required(VERSION 2.8)
project(tonav)
set (tonav_CMAKE_DIR "${tonav_SOURCE_DIR}/cmake")
set (CMAKE_MODULE_PATH ${tonav_CMAKE_DIR} ${CMAKE_MODULE_PATH})

find_package(catkin REQUIRED
             COMPONENTS message_generation std_msgs sensor_msgs image_transport)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs sensor_msgs tf2 roscpp
    DEPENDS eigen opencv
)

include_directories("${tonav_SOURCE_DIR}/include")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -pedantic")

set(SOURCE_FILES
        src/main.cpp
        src/calibration.cpp
        include/calibration.h
        include/filter.h
        include/exceptions/calibration_file_error.h
        src/exceptions/calibration_file_error.cpp
        include/exceptions/impossible_exception.h
        src/exceptions/impossible_exception.cpp
        include/exceptions/base_exception.h
        src/exceptions/base_exception.cpp
        include/camera_feed.h
        src/camera_feed.cpp
        include/imu_feed.h
        src/imu_feed.cpp include/imu_item.h
        include/imu_device.h
        src/imu_device.cpp
        include/exceptions/general_exception.h
        src/exceptions/general_exception.cpp
        src/imu_item.cpp
        include/camera_item.h
        src/camera_item.cpp
        include/navigator.h
        src/navigator.cpp
        include/imu_buffer.h
        src/imu_buffer.cpp
        src/filter.cpp
        include/filter_state.h
        src/filter_state.cpp include/feature_tracker.h src/feature_tracker.cpp)
add_executable(tonav ${SOURCE_FILES})

find_package(Eigen3 REQUIRED)
include_directories (AFTER "${EIGEN3_INCLUDE_DIR}")

find_package(Boost REQUIRED COMPONENTS program_options filesystem)
include_directories (AFTER "${Boost_INCLUDE_DIRS}")
target_link_libraries(tonav ${Boost_LIBRARIES})

find_package(OpenCV REQUIRED)
message (STATUS "${OpenCV_INCLUDE_DIRS}")
message (STATUS "${OpenCV_LIBS}")
include_directories (AFTER "${OpenCV_INCLUDE_DIRS}")
target_link_libraries(tonav ${OpenCV_LIBS})

find_package(image_transport REQUIRED)
target_link_libraries(tonav ${image_transport_LIBRARIES})

find_package(cv_bridge REQUIRED)
target_link_libraries(tonav ${cv_bridge_LIBRARIES})

find_package(tf2_ros REQUIRED)
target_link_libraries(tonav ${tf2_ros_LIBRARIES})

cmake_minimum_required(VERSION 2.8)
project(tonav)
set (tonav_CMAKE_DIR "${tonav_SOURCE_DIR}/cmake")
set (CMAKE_MODULE_PATH ${tonav_CMAKE_DIR} ${CMAKE_MODULE_PATH})

find_package(catkin REQUIRED
             COMPONENTS message_generation std_msgs sensor_msgs image_transport)

include_directories("${tonav_SOURCE_DIR}/include")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

set (CMAKE_INSTALL_RPATH /opt/ros/jade/lib)

set(SOURCE_FILES
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
        src/filter_state.cpp
        include/feature_tracker.h
        src/feature_tracker.cpp
        include/frame_features.h
        src/frame_features.cpp
        include/camera_pose.h
        include/feature_track.h
        src/feature_track.cpp
        src/camera_pose.cpp
        include/tonav.h
        src/tonav.cpp
        include/tonav_ros.h
        src/tonav_ros.cpp
        include/body_state.h
        src/body_state.cpp
        include/ring_buffer.h
        src/ring_buffer.cpp
)
add_executable(tonav src/main.cpp ${SOURCE_FILES})

find_package(Eigen3 REQUIRED)
include_directories (AFTER "${EIGEN3_INCLUDE_DIR}")

find_package(GTest)
if (${GTEST_FOUND})
  set (TEST_FILES
    tests/ring_buffer.cpp
  )
  enable_testing()
  include_directories(${GTEST_INCLUDE_DIRS})

  add_executable(tonav-test ${SOURCE_FILES} ${TEST_FILES})
  target_link_libraries(tonav-test ${GTEST_BOTH_LIBRARIES})
  add_test(TestsTonavAll tonav-test)
endif()


catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs sensor_msgs tf2 tf2_ros roscpp
    DEPENDS eigen
)

find_package(Boost REQUIRED COMPONENTS program_options filesystem)
include_directories (AFTER "${Boost_INCLUDE_DIRS}")
target_link_libraries(tonav ${Boost_LIBRARIES})
if (${GTEST_FOUND})
  target_link_libraries(tonav-test ${Boost_LIBRARIES})
endif ()

find_package(OpenCV REQUIRED)
message (STATUS "${OpenCV_INCLUDE_DIRS}")
message (STATUS "${OpenCV_LIBS}")
include_directories (AFTER "${OpenCV_INCLUDE_DIRS}")
target_link_libraries(tonav ${OpenCV_LIBS})
if (${GTEST_FOUND})
  target_link_libraries(tonav-test ${OpenCV_LIBS})
endif ()

find_package(image_transport REQUIRED)
include_directories (AFTER ${image_transport_INCLUDE_DIRS})
target_link_libraries(tonav ${image_transport_LIBRARIES})
if (${GTEST_FOUND})
  target_link_libraries(tonav-test ${image_transport_LIBRARIES})
endif ()

find_package(cv_bridge REQUIRED)
target_link_libraries(tonav ${cv_bridge_LIBRARIES})
if (${GTEST_FOUND})
  target_link_libraries(tonav-test ${cv_bridge_LIBRARIES})
endif ()

find_package(tf2_ros REQUIRED)
target_link_libraries(tonav ${tf2_ros_LIBRARIES})
if (${GTEST_FOUND})
  target_link_libraries(tonav-test ${tf2_ros_LIBRARIES})
endif ()

find_package(Doxygen 1.8.6)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/deploy.sh.in ${CMAKE_CURRENT_BINARY_DIR}/deploy.sh @ONLY)
    add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif(DOXYGEN_FOUND)


cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

set(TONAV_SRC
        src/exceptions/base_exception.cpp
        src/exceptions/calibration_file_error.cpp
        src/exceptions/general_exception.cpp
        src/exceptions/impossible_exception.cpp
        src/body_state.cpp
        src/calibration.cpp
        src/camera_algorithms.cpp
        src/camera_item.cpp
        src/camera_pose.cpp
        src/camera_reprojection_functor.cpp
        src/debug_logger.cpp
        src/feature_id.cpp
        src/feature_rezidualization_result.cpp
        src/feature_track.cpp
        src/feature_tracker.cpp
        src/filter.cpp
        src/filter_state.cpp
        src/frame_features.cpp
        src/geometry.cpp
        src/imu_buffer.cpp
        src/imu_item.cpp
        src/quaternion.cpp
        src/quaternion_tools.cpp
        src/state_initializer.cpp
        src/tonav.cpp)

set(TONAV_HEADERS
        include/exceptions/base_exception.h
        include/exceptions/calibration_file_error.h
        include/exceptions/general_exception.h
        include/exceptions/impossible_exception.h
        include/body_state.h
        include/calibration.h
        include/camera_algorithms.h
        include/camera_item.h
        include/camera_pose.h
        include/camera_reprojection_functor.h
        include/debug_logger.h
        include/feature_id.h
        include/feature_rezidualization_result.h
        include/feature_track.h
        include/feature_tracker.h
        include/filter.h
        include/filter_state.h
        include/frame_features.h
        include/geometry.h
        include/imu_buffer.h
        include/imu_item.h
        include/quaternion.h
        include/quaternion_tools.h
        include/state_initializer.h
        include/tonav.h)

add_library(tonav
    ${TONAV_SRC} ${TONAV_HEADERS}
)
set_property(TARGET tonav PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories (tonav PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
if(UNIX AND NOT APPLE)
    target_link_libraries(tonav LINK_PUBLIC pthread)
endif()

set_target_properties(tonav PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

find_package(Eigen3 REQUIRED)
target_include_directories (tonav PUBLIC "${EIGEN3_INCLUDE_DIR}")

find_package(OpenCV 3 REQUIRED core calib3d features2d imgproc)
target_include_directories (tonav PUBLIC "${OpenCV_INCLUDE_DIRS}")
target_link_libraries(tonav LINK_PUBLIC ${OpenCV_LIBS})

#enable_testing()
#find_package(GTest REQUIRED)

#add_executable(tonavtests
#        test/circular_buffer_test.cpp
#)
#set_target_properties(tonavtests PROPERTIES
#        CXX_STANDARD 11
#        CXX_STANDARD_REQUIRED ON
#        CXX_EXTENSIONS OFF
#)
#target_link_libraries(tonavtests
#        LINK_PUBLIC tonav
#        LINK_PRIVATE ${GTEST_BOTH_LIBRARIES}
#)

target_include_directories(tonav PUBLIC "../thirdparty/nlohmann_json")

add_test(AllTonavTests tonavtests)

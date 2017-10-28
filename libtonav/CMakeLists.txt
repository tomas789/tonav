cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

add_library(tonav
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
        src/stats.cpp
        src/stats_node.cpp
        src/stats_timer.cpp
        src/tonav.cpp
)
target_include_directories (tonav PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

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

enable_testing()
find_package(GTest REQUIRED)

add_executable(tonavtests
        test/circular_buffer_test.cpp
)
set_target_properties(tonavtests PROPERTIES
        CXX_STANDARD 11
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
)
target_link_libraries(tonavtests
        LINK_PUBLIC tonav
        LINK_PRIVATE ${GTEST_BOTH_LIBRARIES}
)

add_test(AllTonavTests tonavtests)
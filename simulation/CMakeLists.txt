set (SRC_FILES
        src/imu/numerical_diff_imu.cpp
        src/odometry/tonav_body_state_odometry.cpp
        src/odometry/tonav_calibration.cpp
        src/trajectory/circular_trajectory.cpp
        src/trajectory/standstill_trajectory.cpp
        src/vision/generated_features_vision.cpp
        src/imu.cpp
        src/main.cpp
        src/odometry.cpp
        src/run_loop.cpp
        src/sim_setup.cpp
        src/sim_setup_component.cpp
        src/trajectory.cpp
        src/vio_simulation.cpp
        src/vision.cpp
)
add_executable(tonav-simulation ${SRC_FILES})
target_include_directories (tonav-simulation PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries (tonav-simulation LINK_PUBLIC tonav)

set_target_properties(tonav-simulation PROPERTIES
        CXX_STANDARD 11
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
)

find_package(OpenCV REQUIRED core viz)
target_include_directories (tonav-simulation PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(tonav-simulation LINK_PUBLIC ${OpenCV_LIBS})

target_include_directories(tonav-simulation PUBLIC "../thirdparty/nlohmann_json")

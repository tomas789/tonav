set (VIO_SIMULATION_SRC_FILES
        src/helpers/kitti_loader_helper.cpp
        src/imu/kitti_imu.cpp
        src/imu/numerical_diff_imu.cpp
        src/odometry/tonav_body_state_odometry.cpp
        src/odometry/tonav_calibration.cpp
        src/odometry/tonav_odometry.cpp
        src/trajectory/circular_trajectory.cpp
        src/trajectory/inplace_rotation_trajectory.cpp
        src/trajectory/kitti_trajectory.cpp
        src/trajectory/standstill_trajectory.cpp
        src/vision/generated_features_vision.cpp
        src/vision/kitti_vision.cpp
        src/helper.cpp
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
set (VIO_SIMULATION_HEADER_FILES
        include/helpers/kitti_loader_helper.h
        include/imu/kitti_imu.h
        include/imu/numerical_diff_imu.h
        include/odometry/tonav_body_state_odometry.h
        include/odometry/tonav_calibration.h
        include/odometry/tonav_odometry.h
        include/trajectory/circular_trajectory.h
        include/trajectory/inplace_rotation_trajectory.h
        include/trajectory/kitti_trajectory.h
        include/trajectory/standstill_trajectory.h
        include/vision/generated_features_vision.h
        include/vision/kitti_vision.h
        include/helper.h
        include/imu.h
        include/odometry.h
        include/run_loop.h
        include/sim_setup.h
        include/sim_setup_component.h
        include/trajectory.h
        include/vio_simulation.h
        include/vision.h
)
add_executable(tonav-simulation ${VIO_SIMULATION_SRC_FILES} ${VIO_SIMULATION_HEADER_FILES})
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

find_package(pybind11 REQUIRED)
add_library(pytonavsimulation MODULE
        src/vio_simulation_py.cpp
        ${VIO_SIMULATION_SRC_FILES}
        ${VIO_SIMULATION_HEADER_FILES}
)
target_include_directories (pytonavsimulation PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories (pytonavsimulation PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../libtonav/include")
target_link_libraries(pytonavsimulation PRIVATE pybind11::module tonav ${OpenCV_LIBS})
set_target_properties(pytonavsimulation PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
        SUFFIX "${PYTHON_MODULE_EXTENSION}")

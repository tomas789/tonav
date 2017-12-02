//
// Created by Tomas Krejci on 11/6/17.
//

#include "vio_simulation.h"
#include "sim_setup.h"
#include "odometry.h"
#include "odometry/tonav_calibration.h"
#include "odometry/tonav_odometry.h"
#include "sim_setup_component.h"

#include "debug_logger.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

template < typename T>
struct BlankDeleter
{
    void operator()(T * inst) const {}
};

PYBIND11_MODULE(pytonavsimulation, m) {
    namespace py = pybind11;
    
    py::class_<tonav::Quaternion>(m, "Quaternion")
        .def(py::init<double, double, double, double>())
        .def_property_readonly("x", &tonav::Quaternion::x)
        .def_property_readonly("y", &tonav::Quaternion::y)
        .def_property_readonly("z", &tonav::Quaternion::z)
        .def_property_readonly("w", &tonav::Quaternion::w)
        .def("coeffs", &tonav::Quaternion::coeffs)
        .def("conjugate", &tonav::Quaternion::conjugate)
        .def("identity", &tonav::Quaternion::identity)
        .def("__repr__", [](const tonav::Quaternion& q) {
            return "Quaternion(...)";
        });
    
    py::class_<SimSetupComponent>(m, "SimSetupComponent");
    
    py::class_<Odometry, SimSetupComponent>(m, "Odometry")
        .def("get_body_position_in_global_frame", &Odometry::getBodyPositionInGlobalFrame)
        .def("get_global_to_body_frame_rotation", &Odometry::getGlobalToBodyFrameRotation)
        .def("get_camera_position_in_global_frame", &Odometry::getCameraPositionInGlobalFrame)
        .def("get_global_to_camera_frame_rotation", &Odometry::getGlobalToCameraFrameRotation);
    
    py::class_<Trajectory, SimSetupComponent>(m, "Trajectory")
        .def("get_body_position_in_global_frame", &Trajectory::getBodyPositionInGlobalFrame)
        .def("get_global_to_body_frame_rotation", &Trajectory::getGlobalToBodyFrameRotation)
        .def("get_camera_position_in_body_frame", &Trajectory::getCameraPositionInGlobalFrame)
        .def("get_body_to_camera_frame_rotation", &Trajectory::getBodyToCameraFrameRotation);
    
    
    py::class_<tonav::Calibration, std::shared_ptr<tonav::Calibration>>(m, "Calibration")
    .def_property("focal_length",
                  &tonav::Calibration::getCameraFocalLength,
                  &tonav::Calibration::setCameraFocalLength)
    .def_property("optical_center",
                  &tonav::Calibration::getCameraOpticalCenter,
                  &tonav::Calibration::setCameraOpticalCenter)
    .def_property("radial_distortion_params",
                  &tonav::Calibration::getCameraRadialDistortionParams,
                  &tonav::Calibration::setCameraRadialDistortionParams)
    .def_property("tangential_distortion_params",
                  &tonav::Calibration::getCameraTangentialDistortionParams,
                  &tonav::Calibration::setCameraTangentialDistortionParams)
    .def_property("camera_delay_time",
                  &tonav::Calibration::getCameraDelayTime,
                  &tonav::Calibration::setCameraDelayTime)
    .def_property("camera_readout_time",
                  &tonav::Calibration::getCameraReadoutTime,
                  &tonav::Calibration::setCameraReadoutTime)
    .def_property("image_noise_variance",
                  &tonav::Calibration::getImageNoiseVariance,
                  &tonav::Calibration::setImageNoiseVariance)
    .def_property_readonly("number_of_features_to_extract",
                           &tonav::Calibration::getNumberOfFeaturesToExtract)
    .def_property("gyroscope_acceleration_sensitivity_matrix",
                  &tonav::Calibration::getGyroscopeAccelerationSensitivityMatrix,
                  &tonav::Calibration::setGyroscopeAccelerationSensitivityMatrix)
    .def_property("gyroscope_shape_matrix",
                  &tonav::Calibration::getGyroscopeShapeMatrix,
                  &tonav::Calibration::setGyroscopeShapeMatrix)
    .def_property("accelerometer_shape_matrix",
                  &tonav::Calibration::getAccelerometerShapeMatrix,
                  &tonav::Calibration::setAccelerometerShapeMatrix)
    .def_property("gyroscope_bias",
                  &tonav::Calibration::getGyroscopeBias,
                  &tonav::Calibration::setGyroscopeBias)
    .def_property("accelerometer_bias",
                  &tonav::Calibration::getAccelerometerBias,
                  &tonav::Calibration::setAccelerometerBias)
    .def_property("global_gravity",
                  &tonav::Calibration::getGlobalGravity,
                  &tonav::Calibration::setGlobalGravity)
    .def_property("accelerometer_variance",
                  &tonav::Calibration::getAccelerometerVariance,
                  &tonav::Calibration::setAccelerometerVariance)
    .def_property("gyroscope_variance",
                  &tonav::Calibration::getGyroscopeVariance,
                  &tonav::Calibration::setGyroscopeVariance)
    .def_property("accelerometer_random_walk_variance",
                  &tonav::Calibration::getAccelerometerRandomWalkVariance,
                  &tonav::Calibration::setAccelerometerRandomWalkVariance)
    .def_property("gyroscope_random_walk_variance",
                  &tonav::Calibration::getGyroscopeRandomWalkVariance,
                  &tonav::Calibration::setGyroscopeRandomWalkVariance)
    .def_property("max_camera_poses",
                  &tonav::Calibration::getMaxCameraPoses,
                  &tonav::Calibration::setMaxCameraPoses)
    .def_property("max_triangulation_iterations",
                  &tonav::Calibration::getMaxTriangulationIterations,
                  &tonav::Calibration::setMaxTriangulationIterations)
    .def_property("orientation_noise",
                  &tonav::Calibration::getOrientationNoise,
                  &tonav::Calibration::setOrientationNoise)
    .def_property("position_noise",
                  &tonav::Calibration::getPositionNoise,
                  &tonav::Calibration::setPositionNoise)
    .def_property("velocity_noise",
                  &tonav::Calibration::getVelocityNoise,
                  &tonav::Calibration::setVelocityNoise)
    .def_property("gyroscope_bias_noise",
                  &tonav::Calibration::getGyroscopeBiasNoise,
                  &tonav::Calibration::setGyroscopeBiasNoise)
    .def_property("accelerometer_bias_noise",
                  &tonav::Calibration::getAccelerometerBiasNoise,
                  &tonav::Calibration::setAccelerometerBiasNoise)
    .def_property("gyroscope_acceleration_sensitivity_noise",
                  &tonav::Calibration::getGyroscopeAccelerationSensitivityMatrixNoise,
                  &tonav::Calibration::setGyroscopeAccelerationSensitivityMatrixNoise)
    .def_property("gyroscope_shape_matrix_noise",
                  &tonav::Calibration::getGyroscopeShapeMatrixNoise,
                  &tonav::Calibration::setGyroscopeShapeMatrixNoise)
    .def_property("accelerometer_shape_matrix_noise",
                  &tonav::Calibration::getAccelerometerShapeMatrixNoise,
                  &tonav::Calibration::setAccelerometerShapeMatrixNoise)
    .def_property("position_of_body_in_camera_frame_noise",
                  &tonav::Calibration::getPositionOfBodyInCameraFrameNoise,
                  &tonav::Calibration::setPositionOfBodyInCameraFrameNoise)
    .def_property("focal_length_noise",
                  &tonav::Calibration::getFocalLengthNoise,
                  &tonav::Calibration::setFocalLengthNoise)
    .def_property("optical_center_noise",
                  &tonav::Calibration::getOpticalCenterNoise,
                  &tonav::Calibration::setOpticalCenterNoise)
    .def_property("radial_distortion_noise",
                  &tonav::Calibration::getRadialDistortionNoise,
                  &tonav::Calibration::setRadialDistortionNoise)
    .def_property("tangential_distortion_noise",
                  &tonav::Calibration::getTangentialDistortionNoise,
                  &tonav::Calibration::setTangentialDistortionNoise)
    .def_property("camera_delay_time_noise",
                  &tonav::Calibration::getCameraDelayTimeNoise,
                  &tonav::Calibration::setCameraDelayTimeNoise)
    .def_property("camera_readout_time_noise",
                  &tonav::Calibration::getCameraReadoutTimeNoise,
                  &tonav::Calibration::setCameraReadoutTimeNoise)
    .def_property("body_to_camera_rotation",
                  &tonav::Calibration::getBodyToCameraRotation,
                  &tonav::Calibration::setBodyToCameraRotation);
    
    py::class_<TonavCalibration, tonav::Calibration, std::shared_ptr<TonavCalibration>>(m, "TonavCalibration")
        .def("get_position_of_body_in_camera_frame", &TonavCalibration::getPositionOfBodyInCameraFrame);
    
    py::class_<TonavOdometry, Odometry>(m, "TonavOdometry")
        .def("get_tonav_calibration", &TonavOdometry::getTonavCalibration);
    
    py::class_<SimSetup, std::shared_ptr<SimSetup>>(m, "SimSetup")
        .def(py::init(&SimSetup::load))
        .def("getTrajectory", (Trajectory& (SimSetup::*)())&SimSetup::getTrajectory, py::return_value_policy::reference_internal)
        .def("getImu", (Imu& (SimSetup::*)())&SimSetup::getImu, py::return_value_policy::reference_internal)
        .def("getVision", (Vision& (SimSetup::*)())&SimSetup::getVision, py::return_value_policy::reference_internal)
        .def("getOdometry", (Odometry& (SimSetup::*)())&SimSetup::getOdometry, py::return_value_policy::reference_internal);
    
    
    
    py::class_<tonav::DebugLogger, std::unique_ptr<tonav::DebugLogger, BlankDeleter<tonav::DebugLogger>>>(m, "DebugLogger")
    .def_static("getInstance", &tonav::DebugLogger::getInstance, py::return_value_policy::reference)
    .def("set_output_file", &tonav::DebugLogger::setOutputFile)
    .def("write_and_clear", &tonav::DebugLogger::writeAndClear);
    
    py::class_<VioSimulation>(m, "VioSimulation")
        .def(py::init<>())
        .def("set_headless", &VioSimulation::setHeadless)
        .def("set_simulation_length", &VioSimulation::setSimulationLength)
        .def("run", &VioSimulation::run);
}

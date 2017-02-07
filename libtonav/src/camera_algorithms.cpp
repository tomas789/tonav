#include "camera_algorithms.h"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <fstream>
#include <unsupported/Eigen/LevenbergMarquardt>

#include "camera_reprojection_functor.h"
#include "feature_track.h"
#include "filter.h"
#include "filter_state.h"

CameraAlgorithms::CameraAlgorithms(const Filter* filter)
: filter_(filter) {
}

Eigen::Vector3d CameraAlgorithms::initialGuessFeaturePosition(const Eigen::Vector2d& z0, const Eigen::Vector2d& z1, const Eigen::Matrix3d& R_C0_C1, const Eigen::Vector3d& p_C1_C0, InitialGuessMethod method) const {
    Eigen::Vector3d v1;
    v1 << z0(0), z0(1), 1.0;
    Eigen::Vector3d v2;
    v2 << z1(0), z1(1), 1.0;
    
    v1.normalize();
    v2.normalize();
    
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> a(3, 2);
    a.block<3, 1>(0, 0) = v1;
    a.block<3, 1>(0, 1) = -1*R_C0_C1*v2;
    Eigen::Vector3d b = p_C1_C0;
    
    Eigen::Vector2d x;
    
    if (method == InitialGuessMethod::SVD) {
        x = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    } else if (method == InitialGuessMethod::QR) {
        x = a.colPivHouseholderQr().solve(b);
    } else if (method == InitialGuessMethod::normal) {
        x = (a.transpose() * a).ldlt().solve(a.transpose() * b);
    } else {
        throw std::invalid_argument("ReprojectionOptimizer::initialGuess method must be either SVD, QR or normal");
    }
    
    return x(0) * v1;
}

std::pair<bool, Eigen::Vector3d> CameraAlgorithms::triangulateGlobalFeaturePosition(
        const FeatureTrack &feature_track) const {
    std::size_t n = feature_track.posesTrackedCount();
    
    std::vector<Eigen::Matrix3d> rotations;
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector2d> measurements;
    
    const CameraPoseBuffer& poses = filter_->state().poses();
    assert(n <= poses.size() - 1);
    CameraPoseBuffer::const_iterator it_c0 = std::next(poses.begin(), poses.size() - (n+1));
    CameraPoseBuffer::const_iterator it_end = std::prev(std::end(poses));
    for (auto it = it_c0; it != it_end; ++it) {
        const CameraPose& c0 = *it_c0;
        const CameraPose& ci = *it;
        
        Quaternion q_Ci_C0 = c0.getRotationToOtherPose(ci, *filter_);
        Eigen::Vector3d p_C0_Ci = ci.getPositionOfAnotherPose(c0, *filter_);
        Eigen::Vector2d z_i = feature_track[it - it_c0];
        
        rotations.push_back(q_Ci_C0.toRotationMatrix());
        positions.push_back(p_C0_Ci);
        measurements.push_back(z_i);
    }
    
    // Initial guess
    const FilterState& state = filter_->state();
    const Eigen::Vector2d& z0 = (feature_track[0].array() - state.optical_center_.array()) / state.focal_point_.array();
    const Eigen::Vector2d& z_last = (feature_track[n-1].array() - state.optical_center_.array()) / state.focal_point_.array();
    const Quaternion q_Clast_C0 = (it_c0)->getRotationToOtherPose(*(it_c0 + (n-1)), *filter_);
    const Eigen::Vector3d& p_Clast_C0 = (it_c0)->getPositionOfAnotherPose(*(it_c0 + (n-1)), *filter_);
    Eigen::Vector3d initial_guess = initialGuessFeaturePosition(z0, z_last, q_Clast_C0.conjugate().toRotationMatrix(), p_Clast_C0, InitialGuessMethod::SVD);
    Eigen::VectorXd x = initial_guess;
    x(2) = 1.0;
    x /= initial_guess(2);
    
    {
        Eigen::VectorXd xs(measurements.size());
        Eigen::VectorXd ys(measurements.size());
        for (std::size_t i = 0; i < measurements.size(); ++i) {
            xs(i) = measurements[i](0);
            ys(i) = measurements[i](1);
        }
        Eigen::Matrix3d R_Clast_C0 = q_Clast_C0.toRotationMatrix();
        Eigen::IOFormat formatter(4, 0, ", ", "\n", "[", "]");
        Eigen::Matrix<double, 9, 1> r;
        r.block<3, 1>(0, 0) = R_Clast_C0.block<1, 3>(0, 0);
        r.block<3, 1>(3, 0) = R_Clast_C0.block<1, 3>(1, 0);
        r.block<3, 1>(6, 0) = R_Clast_C0.block<1, 3>(2, 0);
        std::ofstream out("~/dump/feature_" + std::to_string(feature_track.getFeatureId()) + ".txt");
        out << "{" << std::endl;
        out << "\"c0_pose_id\": " << it_c0->getCameraPoseId() << "," << std::endl;
        out << "\"z0\": " << z0.transpose().format(formatter) << "," << std::endl;
        out << "\"z_last\": " << z_last.transpose().format(formatter) << "," << std::endl;
        out << "\"p_Clast_C0\": " << p_Clast_C0.transpose().format(formatter) << "," << std::endl;
        out << "\"R_Clast_C0\": " << r.transpose().format(formatter) << "," << std::endl;
        out << "\"xs\": " << xs.transpose().format(formatter) << "," << std::endl;
        out << "\"ys\": " << ys.transpose().format(formatter) << "," << std::endl;
        out << "\"initial_guess\": " << initial_guess.transpose().format(formatter) << std::endl;
        out << "}" << std::endl;
    }
    
    //    std::cout << " ⛄ " << std::endl;
    //    std::cout << "p_B0_G: [" << it_c0->getBodyPositionInGlobalFrame().transpose() << "]^T" << std::endl;
    //    std::cout << "p_Blast_G: [" << (it_c0 + (n-1))->getBodyPositionInGlobalFrame().transpose() << "]^T" << std::endl;
    //    std::cout << "distance: " << (it_c0->getBodyPositionInGlobalFrame() - (it_c0 + (n-1))->getBodyPositionInGlobalFrame()).norm() << " m" << std::endl;
    //    std::cout << "p_C0_G: [" << it_c0->getCameraPositionInGlobalFrame(*this).transpose() << "]^T" << std::endl;
    //    std::cout << "p_Clast_G: [" << (it_c0 + (n-1))->getCameraPositionInGlobalFrame(*this).transpose() << "]^T" << std::endl;
    //    std::cout << "distance: " << (it_c0->getCameraPositionInGlobalFrame(*this) - (it_c0 + (n-1))->getCameraPositionInGlobalFrame(*this)).norm() << " m" << std::endl;
    //    std::cout << "p_Clast_C0: [" << (it_c0)->getPositionOfAnotherPose(*(it_c0 + (n-1)), *this).transpose() << "]^T" << std::endl;
    //    std::cout << "||p_Clast_C0||: " << (it_c0)->getPositionOfAnotherPose(*(it_c0 + (n-1)), *this).norm() << " m" << std::endl;
    //    std::cout << " ⛸ ⛸ ⛸ ⛸ " << std::endl;
    
    // Solve
    CameraReprojectionFunctor functor(rotations, positions, measurements, *filter_);
    Eigen::NumericalDiff<CameraReprojectionFunctor> num_diff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<CameraReprojectionFunctor>> lm(num_diff);
    Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(x);
    
    std::string msg;
    switch (status) {
        case Eigen::LevenbergMarquardtSpace::Status::NotStarted:
            msg = "NotStarted";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::Running:
            msg = "Running";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::ImproperInputParameters:
            msg = "ImproperInputParameters";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::RelativeReductionTooSmall:
            msg = "RelativeReductionTooSmall";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::RelativeErrorTooSmall:
            msg = "RelativeErrorTooSmall";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::RelativeErrorAndReductionTooSmall:
            msg = "RelativeErrorAndReductionTooSmall";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::CosinusTooSmall:
            msg = "CosinusTooSmall";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::TooManyFunctionEvaluation:
            msg = "TooManyFunctionEvaluation";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::FtolTooSmall:
            msg = "FtolTooSmall";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::XtolTooSmall:
            msg = "XtolTooSmall";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::GtolTooSmall:
            msg = "GtolTooSmall";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::UserAsked:
            msg = "UserAsked";
            break;
            
        default:
            msg = "UNKNOWN";
            break;
    }
    std::string status_msg;
    switch (lm.info()) {
        case Eigen::ComputationInfo::Success:
            status_msg = "Success";
            break;
        case Eigen::ComputationInfo::NumericalIssue:
            status_msg = "NumericalIssue";
            break;
        case Eigen::ComputationInfo::NoConvergence:
            status_msg = "NoConvergence";
            break;
        case Eigen::ComputationInfo::InvalidInput:
            status_msg = "InvalidInput";
            break;
            
        default:
            break;
    }
    // std::cout << "Optimization ended after " << lm.iterations() << " iteration(s) with status " << status_msg << " (" << msg << ")" << std::endl;
    
    Eigen::VectorXd fvec;
    fvec.resize(functor.values());
    functor(x, fvec);
    
    Eigen::VectorXd fvec_inverted;
    fvec_inverted.resize(functor.values());
    functor(-1*x, fvec_inverted);
    
    Quaternion q_G_C0 = it_c0->getCameraOrientationInGlobalFrame(*filter_).conjugate();
    Eigen::Vector3d p_C0_G = it_c0->getCameraPositionInGlobalFrame(*filter_);
    Eigen::Vector3d param;
    param << x(0), x(1), 1.0;
    double rho = x(2);
    
    Eigen::Vector3d global_position = 1.0/rho * q_G_C0.toRotationMatrix()*param + p_C0_G;
    if (std::isnan(global_position.norm())) {
        throw std::runtime_error("Global feature position is NaN");
    }
    
    return std::make_pair(fvec.maxCoeff() <= 1000, global_position);
}

Eigen::Vector2d CameraAlgorithms::cameraProject(const Eigen::Vector3d& p) const {
    double u = p(0) / p(2);
    double v = p(1) / p(2);
    Eigen::Vector2d uv_vec;
    uv_vec << u, v;
    double r = u*u + v*v;
    const FilterState& state = filter_->state();
    double k_1 = state.radial_distortion_[0];
    double k_2 = state.radial_distortion_[1];
    double k_3 = state.radial_distortion_[2];
    double t_1 = state.tangential_distortion_[0];
    double t_2 = state.tangential_distortion_[1];
    double d_r = 1 + k_1*r + k_2*r*r + k_3*r*r*r;
    Eigen::Vector2d d_t;
    d_t <<
    2*u*v*t_1 + (r + 2*u*u)*t_2,
    2*u*v*t_2 + (r + 2*v*v)*t_1;
    return state.optical_center_ + state.focal_point_.asDiagonal()*(d_r*uv_vec + d_t);
}

Eigen::Matrix<double, 2, 3> CameraAlgorithms::cameraProjectJacobian(const Eigen::Vector3d& p) const {
    double x = p(0);
    double y = p(1);
    double z = p(2);
    double u = x / z;
    double v = y / z;
    double r = u*u + v*v;
    const FilterState& state = filter_->state();
    double k1 = state.radial_distortion_(0);
    double k2 = state.radial_distortion_(1);
    double k3 = state.radial_distortion_(2);
    double t1 = state.tangential_distortion_(0);
    double t2 = state.tangential_distortion_(1);
    double dr = 1.0 + k1*r + k2*r*r + k3*std::pow(r, 3.0);






    Eigen::Matrix<double, 2, 3> d;
    d(0, 0) = state.focal_point_(0) / z;
    d(0, 1) = 0;
    d(0, 2) = -state.focal_point_(0)*x/(z*z);
    d(1, 0) = 0;
    d(1, 1) = state.focal_point_(1) / z;
    d(1, 2) = -state.focal_point_(1)*y/(z*z);
    return d;





    
    Eigen::RowVector3d uv_by_xyz;
    uv_by_xyz << y/(z*z), x/(z*z), -2.0*x*y/std::pow(z, 3.0);
    
    Eigen::RowVector3d u_by_xyz;
    u_by_xyz << 1.0/z, 0.0, -1.0*x/(z*z);
    
    Eigen::RowVector3d v_by_xyz;
    v_by_xyz << 0.0, 1.0/z, -1.0*y/(z*z);
    
    double x2_y2 = x*x + y*y;
    
    Eigen::RowVector3d r_by_xyz;
    r_by_xyz << 2.0*x/(z*z), 2.0*y/(z*z), -2.0/std::pow(z, 3.0)*(x2_y2);
    
    Eigen::RowVector3d r_squared_by_xyz;
    r_squared_by_xyz << 4.0*x*(x2_y2)/std::pow(z, 4.0), 4.0*y*x2_y2/std::pow(z, 4.0), -4.0*x2_y2*x2_y2/std::pow(z, 5.0);
    
    Eigen::RowVector3d r_cubed_by_xyz;
    r_cubed_by_xyz << 6.0*x*x2_y2*x2_y2/std::pow(z, 6.0), 6.0*y*x2_y2*x2_y2/std::pow(z, 6.0), -6.0*std::pow(x2_y2, 3.0)/std::pow(z, 7.0);
    
    Eigen::RowVector3d dr_by_xyz = k1*r_by_xyz + k2*r_squared_by_xyz + k3*r_cubed_by_xyz;
    
    Eigen::Matrix<double, 2, 3> dt_by_xyz;
    dt_by_xyz.block<1, 3>(0, 0) = 2.0*t1*uv_by_xyz + t2*r_by_xyz + 4.0*t2*u*u_by_xyz;
    dt_by_xyz.block<1, 3>(1, 0) = 2.0*t2*uv_by_xyz + t1*r_by_xyz + 4.0*t1*v*v_by_xyz;
    
    Eigen::Matrix2d lhs = state.focal_point_.asDiagonal();
    
    Eigen::Matrix<double, 2, 3> rhs;
    rhs(0, 0) = dr/z + dr_by_xyz(0)*u + dt_by_xyz(0);
    rhs(0, 1) = dr_by_xyz(1)*u + dt_by_xyz(1);
    rhs(0, 2) = -1.0*dr/z*u + dr_by_xyz(2)*u + dt_by_xyz(2);
    rhs(1, 0) = dr_by_xyz(0)*v + dt_by_xyz(0);
    rhs(1, 1) = dr/z + dr_by_xyz(1)*v + dt_by_xyz(1);
    rhs(1, 2) = -1.0*dr/z*v + dr_by_xyz(2)*v + dt_by_xyz(2);
    
    return lhs*rhs;
}

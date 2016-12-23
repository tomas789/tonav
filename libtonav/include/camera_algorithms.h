#ifndef TONAV_CAMERA_ALGORITHMS_H
#define TONAV_CAMERA_ALGORITHMS_H

#include <Eigen/Core>

class Filter;
class FeatureTrack;

enum InitialGuessMethod {
    SVD, QR, normal
};

class CameraAlgorithms {
public:
    CameraAlgorithms(const Filter* filter_);
    
    /**
     * @brief Initial guess of global feature position from its two measurements.
     */
    Eigen::Vector3d initialGuessFeaturePosition(const Eigen::Vector2d& z0, const Eigen::Vector2d& z1,
            const Eigen::Matrix3d& R_C1_C0, const Eigen::Vector3d& p_C1_C0, InitialGuessMethod method) const;
    
    /**
     * @brief Calculate \f$ {}^{G}\mathbf{p}_{\mathbf{f}_i} \f$
     *
     * This estimates feature position in global frame. It uses Gauss-Newton algorithm (or Levenberg-Marquardt
     * algorithm) to achieve this. Feature location is described using inverse depth parametrization of feature in
     * first camera pose that captured it.
     *
     * @return Estimate for \f$ {}^{G}\mathbf{p}_{\mathbf{f}_i} \f$.
     */
    std::pair<bool, Eigen::Vector3d> triangulateGlobalFeaturePosition(const FeatureTrack& feature_track) const;
    
    /**
     * @brief Transform inverse depth parametrized estimate of feature's
     *        position in \f$C_0\f$ to \f$C_i\f$.
     *
     * It is implementation of function \f$\mathbf{g}_i\f$.
     */
    Eigen::Vector3d transformFeatureInverseDepthEstimateToCameraFrame(const FeatureTrack& feature_track, std::size_t i,
            const Eigen::Vector3d& est) const;
    
    /**
     * @brief Implementation of camera model.
     *
     * It implements pinhole camera model with radial and tangential
     * distortions;
     *
     * It is implementation of function \f$\mathbf{h}\f$
     */
    Eigen::Vector2d cameraProject(const Eigen::Vector3d& p) const;
    
    /**
     * @brief Compute jacobian of camera projection function.
     */
    Eigen::Matrix<double, 2, 3> cameraProjectJacobian(const Eigen::Vector3d& p) const;
    
private:
    const Filter* filter_;
};

#endif //TONAV_CAMERA_ALGORITHMS_H

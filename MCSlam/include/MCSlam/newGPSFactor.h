#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

/**
 * GPS position factor that also estimates the VIO-to-ENU alignment.
 *
 * Connects the body pose V_T_B (in the VIO frame) with the alignment transform
 * E_T_V (VIO frame to local ENU frame). The measurement model is
 *     h(V_T_B, E_T_V) = E_T_V * (V_T_B * B_t_G)
 * where B_t_G is the GPS antenna lever arm in the body frame, and the error is
 * h - gps_E (the GPS fix expressed in the local ENU frame).
 */
class newGPSFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>;
    using Base::evaluateError;

    newGPSFactor(gtsam::Key pose_key, const gtsam::Point3& gps_E, gtsam::Key alignment_key,
                 const gtsam::SharedNoiseModel& model, const gtsam::Pose3& body_P_sensor)
        : Base(model, pose_key, alignment_key), gps_E_(gps_E), body_P_sensor_(body_P_sensor) {}

    ~newGPSFactor() override = default;

    /// Error h(V_T_B, E_T_V) - gps_E with Jacobians H1 = d/dV_T_B, H2 = d/dE_T_V.
    gtsam::Vector evaluateError(const gtsam::Pose3& V_T_B, const gtsam::Pose3& E_T_V,
                                gtsam::OptionalMatrixType H1,
                                gtsam::OptionalMatrixType H2) const override;

    void print(const std::string& s = "",
               const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

private:
    gtsam::Point3 gps_E_;        // GPS fix in the local ENU frame
    gtsam::Pose3 body_P_sensor_; // GPS antenna extrinsic; only the translation is used
};

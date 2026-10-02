#include "MCSlam/newGPSFactor.h"

#include <iostream>

using gtsam::Matrix;
using gtsam::Matrix36;
using gtsam::Matrix3;
using gtsam::Point3;

gtsam::Vector newGPSFactor::evaluateError(const gtsam::Pose3& V_T_B, const gtsam::Pose3& E_T_V,
                                          gtsam::OptionalMatrixType H1,
                                          gtsam::OptionalMatrixType H2) const {
    // Chain rule through the composition E_T_V * (V_T_B * B_t_G):
    //   antenna_V = V_T_B.transformFrom(B_t_G)      d/dV_T_B   = H_pose      (3x6)
    //   antenna_E = E_T_V.transformFrom(antenna_V)  d/dE_T_V   = H2          (3x6)
    //                                               d/dantenna = H_point     (3x3)
    //   H1 = H_point * H_pose
    Matrix36 H_pose;
    Matrix3 H_point;
    const Point3 antenna_V = V_T_B.transformFrom(body_P_sensor_.translation(), H1 ? &H_pose : nullptr);
    const Point3 antenna_E = E_T_V.transformFrom(antenna_V, H2, H1 ? &H_point : nullptr);
    if (H1) *H1 = H_point * H_pose;
    return antenna_E - gps_E_;
}

void newGPSFactor::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const {
    std::cout << (s.empty() ? "" : s + " ") << "newGPSFactor on " << keyFormatter(key<1>())
              << " connected to " << keyFormatter(key<2>()) << "\n"
              << "  GPS measurement (ENU): " << gps_E_.transpose() << "\n";
    noiseModel_->print("  noise model: ");
}

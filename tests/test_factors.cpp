#include <gtest/gtest.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "MCSlam/GtsamFactorHelpers.h"
#include "MCSlam/newGPSFactor.h"

using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::symbol_shorthand::E;
using gtsam::symbol_shorthand::X;

namespace {

void expectJacobiansMatchNumerical(const gtsam::NoiseModelFactor& factor, const gtsam::Values& values) {
    // Compares analytic Jacobians against central differences.
    EXPECT_TRUE(gtsam::internal::testFactorJacobians("factor", factor, values, 1e-6, 1e-6));
}

}  // namespace

TEST(NewGPSFactor, ZeroErrorAtConsistentState) {
    const Pose3 V_T_B(Rot3::Ypr(0.3, -0.1, 0.05), Point3(2.0, -1.0, 0.5));
    const Pose3 E_T_V(Rot3::Yaw(1.1), Point3(100.0, 50.0, -3.0));
    const Pose3 B_P_G(Rot3(), Point3(0.2, 0.0, 0.8));
    const Point3 gps = E_T_V.transformFrom(V_T_B.transformFrom(B_P_G.translation()));

    const newGPSFactor f(X(0), gps, E(0), gtsam::noiseModel::Isotropic::Sigma(3, 0.5), B_P_G);
    EXPECT_LT(f.evaluateError(V_T_B, E_T_V).norm(), 1e-9);
}

TEST(NewGPSFactor, JacobiansMatchNumerical) {
    const newGPSFactor f(X(0), Point3(101.0, 49.0, -2.0), E(0),
                         gtsam::noiseModel::Isotropic::Sigma(3, 0.5), Pose3(Rot3(), Point3(0.2, 0.0, 0.8)));
    gtsam::Values v;
    v.insert(X(0), Pose3(Rot3::Ypr(0.3, -0.1, 0.05), Point3(2.0, -1.0, 0.5)));
    v.insert(E(0), Pose3(Rot3::Yaw(1.1), Point3(100.0, 50.0, -3.0)));
    expectJacobiansMatchNumerical(f, v);
}

TEST(RigResectioningFactor, JacobiansMatchNumericalWithAndWithoutExtrinsic) {
    auto K = std::make_shared<gtsam::Cal3_S2>(458.0, 457.0, 0.0, 367.0, 248.0);
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
    const Point3 landmark(0.4, -0.3, 5.0);
    const Pose3 pose(Rot3::Ypr(0.05, 0.02, -0.03), Point3(0.1, 0.0, -0.2));
    gtsam::Values v;
    v.insert(X(0), pose);

    const RigResectioningFactor plain(noise, X(0), K, gtsam::Point2(380.0, 230.0), landmark, 0, 0);
    expectJacobiansMatchNumerical(plain, v);

    const Pose3 body_P_cam(Rot3::Ypr(0.0, 0.0, 0.01), Point3(0.11, 0.0, 0.0));
    const RigResectioningFactor rig(noise, X(0), K, gtsam::Point2(380.0, 230.0), landmark, 0, 1, body_P_cam);
    expectJacobiansMatchNumerical(rig, v);
}

TEST(RigResectioningFactor, PointBehindCameraIsPenalizedNotThrown) {
    auto K = std::make_shared<gtsam::Cal3_S2>(458.0, 457.0, 0.0, 367.0, 248.0);
    const RigResectioningFactor f(gtsam::noiseModel::Isotropic::Sigma(2, 1.0), X(0), K,
                                  gtsam::Point2(380.0, 230.0), Point3(0.0, 0.0, -5.0), 0, 0);
    gtsam::Matrix H;
    const gtsam::Vector err = f.evaluateError(Pose3(), &H);
    EXPECT_DOUBLE_EQ(err(0), 2.0 * K->fx());
    EXPECT_TRUE(H.isZero());
}

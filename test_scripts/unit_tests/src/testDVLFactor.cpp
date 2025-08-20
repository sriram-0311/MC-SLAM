//
// Created by pushyami kaveti on 5/29/24.
//

/**

A simple unit test to check the evaluate error output and jacobians of the DVl factor

**/

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>
#include "new_DVL_factor.h"

using namespace std;
using namespace gtsam;
using namespace std::placeholders;

TEST(newDVLFactor, basictest){
    //

    double E = 1;
    double N = 1;
    double U = 1;


    Key pose_key(1);
    Key vel_key(2);
    SharedNoiseModel meas_noise = noiseModel::Isotropic::Sigma(3, 0.25);
    gtsam::Vector3 dvl_meas = gtsam::Vector3(0.1, 0, 0.0);
    gtsam::Vector3 ang_vel_measured = gtsam::Vector3(0,0,0);
    gtsam::Pose3 T_bd = Pose3(gtsam::Rot3(), gtsam::Point3(0, -0.5, 0));
    DVLFactor factor(pose_key,vel_key, dvl_meas, ang_vel_measured, meas_noise, T_bd);

    // create linearization point at zero error
    const Pose3 pose_i = Pose3();
    const Vector3 vel_i = Vector3(0.1, 0, 0);
    const Vector err = factor.evaluateError(pose_i, vel_i, boost::none, boost::none);
    const double tol{1e-5};
    EXPECT(assert_equal(Z_3x1, err,tol ));

    // Calculate numerical derivatives
    Matrix numericalH1 = numericalDerivative21<Vector3, Pose3, Vector3>(
            std::bind(&DVLFactor::evaluateError, factor, std::placeholders::_1, std::placeholders::_2, boost::none, boost::none),
            pose_i,
            vel_i
    );
    Matrix numericalH2 = numericalDerivative22<Vector3, Pose3, Vector3>(
            std::bind(&DVLFactor::evaluateError, factor, std::placeholders::_1, std::placeholders::_2, boost::none, boost::none),
            pose_i,
            vel_i
    );

    // Analytic and numerical derivative
    Matrix actualH1, actualH2;
    factor.evaluateError(pose_i, vel_i, actualH1, actualH2);

    // Compare numerical and analytical
    EXPECT(assert_equal(numericalH1, actualH1));
    EXPECT(assert_equal(numericalH2, actualH2));



}

int main() {
    TestResult result;
    TestRegistry::runAllTests(result);

    std::cout << "Failed: " << result.getFailureCount();
    return 0;
}


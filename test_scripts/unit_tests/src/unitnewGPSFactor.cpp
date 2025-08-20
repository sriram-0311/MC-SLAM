/**

A simple unit test to check the evaluate error output and jacobians of the newGPSFactor

**/




#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include "newGPSFactor.h"
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>


using namespace std;
using namespace gtsam;
using namespace std::placeholders;


TEST(newGPSFactor, Constructor){

    double E = 1;
    double N = 1;
    double U = 1;


    Key key(1);
    Key transform_key(2);
    SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
    newGPSFactor factor(key, gtsam::Point3(E, N, U), transform_key, model, gtsam::Pose3());

    // create linearization point at zero error
    const Pose3 vins_pose(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(E, N, U));
    const Pose3 E_T_V(Rot3(), Point3(0, 0, 0));

    EXPECT(assert_equal(Z_3x1, factor.evaluateError(vins_pose, E_T_V, boost::none, boost::none)));

    // Calculate numerical derivatives
    Matrix numericalH1 = numericalDerivative21<Vector3, Pose3, Pose3>(
            std::bind(&newGPSFactor::evaluateError, factor, std::placeholders::_1, std::placeholders::_2, boost::none, boost::none),
            vins_pose,
            E_T_V
    );
    Matrix numericalH2 = numericalDerivative22<Vector3, Pose3, Pose3>(
            std::bind(&newGPSFactor::evaluateError, factor, std::placeholders::_1, std::placeholders::_2, boost::none, boost::none),
            vins_pose,
            E_T_V
    );

    // Analytic and numerical derivative
    Matrix actualH1, actualH2;
    factor.evaluateError(vins_pose, E_T_V, actualH1, actualH2);

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


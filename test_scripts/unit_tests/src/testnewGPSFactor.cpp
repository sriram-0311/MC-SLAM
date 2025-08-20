/**
* A simple test with a 2d robot.
 * We have dummy GPS measurements and dummy odometry measurements that act as the VINS output
 *
*/

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <glog/logging.h>
#include <gtsam/inference/Symbol.h>
#include "newGPSFactor.h"

using namespace std;
using namespace gtsam;


int main() {

    NonlinearFactorGraph graph;

    // noise - set to zero
    gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << gtsam::Vector3::Constant(0.0), gtsam::Vector3::Constant(0.0)).finished());
    gtsam::noiseModel::Diagonal::shared_ptr gpsNoise = noiseModel::Diagonal::Sigmas(Vector3(0.0, 0.0, 0.0));


    // extrinsic - identity
    gtsam::Pose3 Tbg_gtsam = Pose3();

    // rotation matrix to rotate by yaw.
    gtsam::Matrix rot_z = gtsam::Rot3::Rz(M_PI_4).matrix();

    // E_T_V
    Pose3 E_T_V(Rot3::Rz(M_PI_4  * -1 ), Point3(1.2, 0.2, 0.01));
    VLOG(0) << "E_T_V " << E_T_V;
    // create dummy vins poses
    gtsam::Pose3 VINS_1(Rot3(), Vector3(0.1, 0.0, 0.0));
    gtsam::Pose3 VINS_2(Rot3(), Vector3(2.1, 0.0, 0.0));
    gtsam::Pose3 VINS_3(Rot3(), Vector3(4.1, 0.0, 0.0));

    gtsam::Point3 GPS_ENU_1(0, 0, 0);
    gtsam::Point3 GPS_ENU_2 = rot_z * gtsam::Vector3(2, 0, 0); // rotated the pointby 45 degrees
    gtsam::Point3 GPS_ENU_3 = rot_z * gtsam::Vector3(4, 0, 0); // rotated the pointby 45 degrees


    graph.emplace_shared<BetweenFactor<Pose3>>(gtsam::Symbol('x', 1), gtsam::Symbol('x', 2),
                                               Pose3(Rot3(), Vector3(2, 0, 0)), poseNoise);
    graph.emplace_shared<BetweenFactor<Pose3>>(gtsam::Symbol('x', 2), gtsam::Symbol('x', 3),
                                              Pose3(Rot3(), Vector3(2, 0, 0)), poseNoise);

    // Add the new gps factor on both the poses
    auto gps_factor1 = newGPSFactor(gtsam::Symbol('x', 1), GPS_ENU_1, gtsam::Symbol('t', 0), gpsNoise, Tbg_gtsam);
    auto gps_factor2 = newGPSFactor(gtsam::Symbol('x', 2), GPS_ENU_2, gtsam::Symbol('t', 0), gpsNoise, Tbg_gtsam);
    auto gps_factor3 = newGPSFactor(gtsam::Symbol('x', 3), GPS_ENU_3, gtsam::Symbol('t', 0), gpsNoise, Tbg_gtsam);
    graph.add(gps_factor1);
    graph.add(gps_factor2);
    graph.add(gps_factor3);

    Values initialEstimate;
    initialEstimate.insert(gtsam::Symbol('x', 1), VINS_1);
    initialEstimate.insert(gtsam::Symbol('x', 2), VINS_2);
    initialEstimate.insert(gtsam::Symbol('x', 3), VINS_3);
    initialEstimate.insert(gtsam::Symbol('t', 0), E_T_V);

    initialEstimate.print("initialEstimate:\n");


    graph.addPrior(gtsam::Symbol('t', 0), E_T_V, noiseModel::Diagonal::Sigmas(
            (Vector(6) << 2, 2, 2, 2, 2, 2)
                    .finished()));
    graph.addPrior(gtsam::Symbol('x', 1), VINS_1, noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1)
                    .finished()));



    gtsam::LevenbergMarquardtParams Lparams;
    Lparams.setVerbosityLM("SUMMARY");
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, Lparams);
    Values result = optimizer.optimize();
    result.print("Final Result:\n");
    Pose3 new_E_T_V = result.at<Pose3>(gtsam::Symbol('t', 0));

    VLOG(0) << "rot_z" << rot_z;
    VLOG(0) << " GPS_ENU1 " << endl << GPS_ENU_1;
    VLOG(0) <<" E_T_V * vins1 " << endl << new_E_T_V * VINS_1.translation();
    VLOG(0) << " GPS_ENU2 " << endl << GPS_ENU_2;
    VLOG(0) <<" E_T_V * vins2 " << endl << (new_E_T_V * VINS_2.translation());
    VLOG(0) << " GPS_ENU3 " << endl << GPS_ENU_3;
    VLOG(0) <<" E_T_V * vins3 " << endl << (new_E_T_V * VINS_3.translation());

}





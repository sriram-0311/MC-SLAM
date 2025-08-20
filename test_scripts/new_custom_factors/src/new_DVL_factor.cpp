//
// Created by auv on 5/27/24.
//
#include "new_DVL_factor.h"

// Evaluate error function
Vector DVLFactor::evaluateError(const Pose3& pose_i, const Vector3& vel_i, boost::optional<Matrix&> H1 , boost::optional<Matrix&> H2) const{

   try{
       Matrix H3 = Matrix::Zero(3,3);
       Matrix H4 = Matrix::Zero(3,3);
       Matrix H5 = Matrix::Zero(3,3);


       Point3 lever_arm_vel = omega.cross(body_P_sensor.translation());
       Point3 v_b =   pose_i.rotation().unrotate(vel_i, H3, H4) + lever_arm_vel; // H3 w.r.t rotation, H4 w.r.t vel
       Point3 v_dvl = body_P_sensor.rotation().unrotate(v_b, boost::none, H5);

       if(H1){
           Matrix rot_jac = H5 * H3;
           *H1 = (gtsam::Matrix(3, 6) << rot_jac, gtsam::Matrix3::Zero()).finished();
           *H2 = H5 * H4;

//           Matrix full_jacobianPose = (gtsam::Matrix(3, 6) << rot_jac, gtsam::Matrix3::Zero()).finished();
//           Matrix full_jacobianVel = H5 * H4;

//           *H1 = full_jacobianPose.topRows<2>();
//           *H2 = full_jacobianVel.topRows<2>();
//           cout << "  DVL errror measurement: " << v_dvl- measured << "\n";
            //cout << "  DVL Jacobians measurement: " << *H2 << "\n";
       }

       Vector3 error = v_dvl- measured; //Vector2(v_dvl(0), v_dvl(1))- measured; //

       return error;
   }
   catch(exception& e){
       if (H1) *H1 = Matrix::Zero(3,6);
       if (H2) *H2 = Matrix::Zero(3,3);
       return Vector3();
   }


}

void DVLFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
    cout << (s.empty() ? "" : s + " ") << "New GPSFactor on " << keyFormatter(key1()) << " connected to " << keyFormatter(key2())  << "\n";
    cout << "  GPS measurement: " << measured << "\n";
    noiseModel_->print("  noise model: ");
}






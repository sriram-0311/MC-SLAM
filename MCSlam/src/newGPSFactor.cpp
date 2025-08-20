#include "MCSlam/newGPSFactor.h"

// Evaluate error function
Vector newGPSFactor::evaluateError(const Pose3& p, const Pose3& E_T_V, boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const{

        // compute the measurement function h(p, etv), so that we can have error = h - gps_P_

        /*
         * E_T_V = transfomation from vins frame to enu frame
         * p = pose of body w.r.t vins frame at time i
         * body_P_sensor = extrinsic between gps and imu
         * H1 = jacobian w.r.t pose - translation
         * H2 = jacobian w.r.t E_T_V
         */

        /*
         * Jacobian Calculation
         *
         * Pose composition: C = C2*C1*C0 = E_T_V * V_P_b * B_t_G
         * We need, dC/dC2 and dC/dC1
         *
         * V_T_G = C1 * C0 = C'
         * H3 = dC'/ dC1
         *
         * measurement = C2 * C'
         * H2 = dC/dC2
         * H4 = dC/dC'
         *
         * chain rule
         * H1 = H4 * H3
         *
         */

        bool print = false;



        Matrix H3 = Matrix::Zero(3,6);
        Matrix H4 = Matrix::Zero(3,3);

        Point3 ext_body_P_sensor = body_P_sensor.translation();

        Point3 V_T_G = p.transformFrom(ext_body_P_sensor, H3);

        Point3 measurement_func = E_T_V.transformFrom(V_T_G, H2, H4);

        if(H1){

            *H1 = H4 * H3;
        }

        Vector3 error = measurement_func - gps_P_;

        if(print){

            VLOG(0) << "p: " << endl << p << endl;
            VLOG(0) << "E_T_V: " << endl << E_T_V << endl;
            VLOG(0) << "ETV * pose " << E_T_V * V_T_G << endl;
            VLOG(0) << "measurement estm: " << endl << measurement_func << endl;
            VLOG(0) << " gps value: " << endl << gps_P_ << endl;
        if(H1){
            VLOG(0) << " *H1 " << endl << *H1 << endl;
            VLOG(0) << " *H2 " << endl << *H2 << endl;
        }
            VLOG(0) << "error: " << endl << error << endl ;

        }





        return error;

}

void newGPSFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
    cout << (s.empty() ? "" : s + " ") << "New GPSFactor on " << keyFormatter(key1()) << " connected to " << keyFormatter(key2())  << "\n";
    cout << "  GPS measurement: " << gps_P_ << "\n";
    noiseModel_->print("  noise model: ");
}

// Test

Vector newGPSFactor2::evaluateError(const gtsam::Pose3 &p, boost::optional<gtsam::Matrix &> H) const {

        VLOG(0) << " initialized " << H.is_initialized() << endl;

        VLOG(0) << "p: " << endl << p << endl;
        Point3 ext_body_P_sensor = body_P_sensor.translation();
        Point3 test_ext(0,0,0);
        Point3 V_T_G = p.transformFrom(test_ext, H);
//        Point3 V_T_G = p.translation(H);
//        cout << " H " << endl << *H << endl;
//        *H =  H1;
        VLOG(0) <<" error is " << V_T_G - gps_P_;

        return V_T_G - gps_P_;


}

void newGPSFactor2::print(const string& s, const KeyFormatter& keyFormatter) const {
        cout << (s.empty() ? "" : s + " ") << "New GPSFactor on " << keyFormatter(key()) <<  "\n";
        cout << "  GPS measurement: " << gps_P_ << "\n";
        noiseModel_->print("  noise model: ");
}




//
// Created by auv on 5/27/24.
//

#ifndef SRC_NEW_DVL_FACTOR_H
#define SRC_NEW_DVL_FACTOR_H

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <glog/logging.h>

using namespace std;
using namespace gtsam;


class DVLFactor: public NoiseModelFactor2<Pose3,  Vector3>{

    Vector3 measured;
    Vector3 omega;
    typedef NoiseModelFactor2<Pose3,  Vector3> Base;

    Pose3 body_P_sensor; // extrinsic as a pose - identity rotation  + extrinsic translation

public:
    // constructor for the factor
    DVLFactor(Key pose_key, Key vel_key, const Vector3& dvl_meas, Vector3& ang_vel_measured, const SharedNoiseModel& model, const Pose3& body_P_sensor) :
            Base(model, pose_key, vel_key), measured(dvl_meas), body_P_sensor(body_P_sensor), omega(ang_vel_measured) {
    }

    ~DVLFactor() override{}

    /** h(x)-z */
    Vector evaluateError(const Pose3& pose_i, const Vector3& vel_i, boost::optional<Matrix&> H1 , boost::optional<Matrix&> H2 ) const override;

    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
        ar & boost::serialization::make_nvp("NoiseModelFactor2",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(measured);
        ar & BOOST_SERIALIZATION_NVP(omega);
        ar & BOOST_SERIALIZATION_NVP(body_P_sensor);
    }

};

#endif //SRC_NEW_DVL_FACTOR_H

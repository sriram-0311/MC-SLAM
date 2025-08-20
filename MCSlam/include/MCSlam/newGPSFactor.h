#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <glog/logging.h>

using namespace std;
using namespace gtsam;


class newGPSFactor: public NoiseModelFactor2<Pose3, Pose3>{


    Point3 gps_P_;
    typedef NoiseModelFactor2<Pose3, Pose3> Base;


    Pose3 body_P_sensor; // extrinsic as a pose - identity rotation  + extrinsic translation

public:
    // constructor for the factor
    newGPSFactor(Key gps_key, const Point3& gps_P_, Key transform_Key, const SharedNoiseModel& model, const Pose3& body_P_sensor) :
            Base(model, gps_key, transform_Key), gps_P_(gps_P_), body_P_sensor(body_P_sensor) {
    }

    ~newGPSFactor() override{}

    /** h(x)-z */
    Vector evaluateError(const Pose3& p, const Pose3& E_T_V, boost::optional<Matrix&> H1 , boost::optional<Matrix&> H2 ) const override;

    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
        ar & boost::serialization::make_nvp("NoiseModelFactor2",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(gps_P_);
    }

};

class newGPSFactor2: public NoiseModelFactor1<Pose3> {

    Point3  gps_P_;
    typedef NoiseModelFactor1<Pose3> Base;

    Pose3 body_P_sensor; // extrinsic as a pose - identity rotation  + extrinsic translation

public:

    // constructor for the factor
    newGPSFactor2(Key gps_key, const Point3& gps_P_, const SharedNoiseModel& model, const Pose3& body_P_sensor):
            Base(model, gps_key), gps_P_(gps_P_), body_P_sensor(body_P_sensor) {
    }

    Vector evaluateError(const Pose3& p,
                         boost::optional<gtsam::Matrix&> H = boost::none) const override;

    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
        ar & boost::serialization::make_nvp("NoiseModelFactor2",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(gps_P_);
    }
};
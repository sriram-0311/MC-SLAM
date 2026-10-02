#include <gtest/gtest.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "MCSlam/Triangulation.h"

namespace {

// Projection matrix P = [R | t] for normalized image coordinates.
cv::Mat makeP(const cv::Vec3d& rvec, const cv::Vec3d& t) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat P(3, 4, CV_64F);
    R.copyTo(P(cv::Rect(0, 0, 3, 3)));
    cv::Mat(t).copyTo(P(cv::Rect(3, 0, 1, 3)));
    return P;
}

cv::Mat project(const cv::Mat& P, const cv::Mat& X3xN) {
    cv::Mat Xh, x;
    mcslam::geom::euclideanToHomogeneous(X3xN, Xh);
    mcslam::geom::homogeneousToEuclidean(P * Xh, x);
    return x;
}

cv::Mat points() {
    return (cv::Mat_<double>(3, 4) << 0.3, -0.5, 1.2, 0.0,
                                       -0.2, 0.4, 0.1, 0.0,
                                        4.0, 6.0, 9.0, 2.5);
}

}  // namespace

TEST(Triangulation, HomogeneousRoundTrip) {
    const cv::Mat X = points();
    cv::Mat Xh, back;
    mcslam::geom::euclideanToHomogeneous(X, Xh);
    ASSERT_EQ(Xh.rows, 4);
    mcslam::geom::homogeneousToEuclidean(Xh * 3.0, back);  // scale must not matter
    EXPECT_LT(cv::norm(back - X), 1e-12);
}

TEST(Triangulation, TwoViewsExact) {
    const cv::Mat X = points();
    const std::vector<cv::Mat> Ps{makeP({0, 0, 0}, {0, 0, 0}), makeP({0, 0.05, 0}, {-0.11, 0, 0})};
    std::vector<cv::Mat> xs;
    for (const auto& P : Ps) xs.push_back(project(P, X));
    cv::Mat Xest;
    mcslam::geom::triangulatePoints(xs, Ps, Xest);
    EXPECT_LT(cv::norm(Xest - X), 1e-9);
}

TEST(Triangulation, NViewsExactAndTypedOverload) {
    const cv::Mat X = points();
    const std::vector<cv::Mat> Ps{makeP({0, 0, 0}, {0, 0, 0}), makeP({0, 0.05, 0}, {-0.11, 0, 0}),
                                  makeP({0.02, -0.03, 0.01}, {0.0, -0.08, 0.02}),
                                  makeP({0, -0.04, 0}, {0.2, 0.01, 0})};
    std::vector<cv::Mat_<double>> xs;
    for (const auto& P : Ps) xs.emplace_back(project(P, X));
    cv::Mat Xest;
    mcslam::geom::triangulatePoints(xs, Ps, Xest);
    EXPECT_LT(cv::norm(Xest - X), 1e-9);
}

TEST(Triangulation, NoisyObservationsStayClose) {
    const cv::Mat X = points();
    const std::vector<cv::Mat> Ps{makeP({0, 0, 0}, {0, 0, 0}), makeP({0, 0.05, 0}, {-0.11, 0, 0}),
                                  makeP({0, -0.04, 0}, {0.2, 0.01, 0})};
    cv::RNG rng(42);
    std::vector<cv::Mat> xs;
    for (const auto& P : Ps) {
        cv::Mat x = project(P, X), noise(x.size(), CV_64F);
        rng.fill(noise, cv::RNG::NORMAL, 0.0, 1e-4);  // ~0.05 px at f=500
        xs.push_back(x + noise);
    }
    cv::Mat Xest;
    mcslam::geom::triangulatePoints(xs, Ps, Xest);
    for (int k = 0; k < X.cols; ++k)
        EXPECT_LT(cv::norm(Xest.col(k) - X.col(k)) / X.at<double>(2, k), 0.01) << "point " << k;
}

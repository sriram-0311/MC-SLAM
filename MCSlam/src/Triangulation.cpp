#include "MCSlam/Triangulation.h"

#include <glog/logging.h>

namespace mcslam::geom {

void homogeneousToEuclidean(const cv::Mat& src, cv::Mat& dst) {
    CHECK_GE(src.rows, 2);
    cv::Mat X;
    src.convertTo(X, CV_64F);
    const int d = X.rows - 1;
    dst.create(d, X.cols, CV_64F);
    for (int c = 0; c < X.cols; ++c) {
        const double w = X.at<double>(d, c);
        for (int r = 0; r < d; ++r) dst.at<double>(r, c) = X.at<double>(r, c) / w;
    }
}

void homogeneousToEuclidean(const cv::Mat& src, cv::Vec3d& dst) {
    CHECK(src.rows == 4 && src.cols == 1) << "expected a 4x1 homogeneous point";
    cv::Mat X;
    homogeneousToEuclidean(src, X);
    dst = cv::Vec3d(X.at<double>(0), X.at<double>(1), X.at<double>(2));
}

void euclideanToHomogeneous(const cv::Mat& src, cv::Mat& dst) {
    cv::Mat X;
    src.convertTo(X, CV_64F);
    cv::vconcat(X, cv::Mat::ones(1, X.cols, CV_64F), dst);
}

namespace {

// Two-view DLT (H&Z 12.2): stack x * P.row(2) - P.row(0), y * P.row(2) - P.row(1).
cv::Vec3d triangulateDLT(const cv::Vec2d& x1, const cv::Vec2d& x2,
                         const cv::Mat& P1, const cv::Mat& P2) {
    cv::Matx44d A;
    for (int j = 0; j < 4; ++j) {
        A(0, j) = x1[0] * P1.at<double>(2, j) - P1.at<double>(0, j);
        A(1, j) = x1[1] * P1.at<double>(2, j) - P1.at<double>(1, j);
        A(2, j) = x2[0] * P2.at<double>(2, j) - P2.at<double>(0, j);
        A(3, j) = x2[1] * P2.at<double>(2, j) - P2.at<double>(1, j);
    }
    cv::Mat X;
    cv::SVD::solveZ(cv::Mat(A), X);
    cv::Vec3d out;
    homogeneousToEuclidean(X, out);
    return out;
}

// N-view DLT: solve [P_i | 0 .. -x_i .. 0] [X; alpha] = 0 for the null vector.
cv::Vec3d triangulateNView(const std::vector<cv::Vec2d>& xs,
                           const std::vector<cv::Mat>& Ps) {
    const int n = static_cast<int>(xs.size());
    cv::Mat design = cv::Mat::zeros(3 * n, 4 + n, CV_64F);
    for (int i = 0; i < n; ++i) {
        Ps[i].copyTo(design(cv::Rect(0, 3 * i, 4, 3)));
        design.at<double>(3 * i + 0, 4 + i) = -xs[i][0];
        design.at<double>(3 * i + 1, 4 + i) = -xs[i][1];
        design.at<double>(3 * i + 2, 4 + i) = -1.0;
    }
    cv::Mat X_and_alphas;
    cv::SVD::solveZ(design, X_and_alphas);
    cv::Vec3d out;
    homogeneousToEuclidean(X_and_alphas.rowRange(0, 4), out);
    return out;
}

}  // namespace

void triangulatePoints(const std::vector<cv::Mat>& points2d,
                       const std::vector<cv::Mat>& projections,
                       cv::Mat& points3d) {
    const size_t views = points2d.size();
    CHECK_GE(views, 2u) << "triangulation needs at least two views";
    CHECK_EQ(views, projections.size());

    std::vector<cv::Mat> xs(views), Ps(views);
    for (size_t v = 0; v < views; ++v) {
        points2d[v].convertTo(xs[v], CV_64F);
        projections[v].convertTo(Ps[v], CV_64F);
        CHECK_EQ(xs[v].rows, 2);
        CHECK_EQ(xs[v].cols, xs[0].cols);
        CHECK(Ps[v].rows == 3 && Ps[v].cols == 4);
    }

    const int n = xs[0].cols;
    points3d.create(3, n, CV_64F);
    std::vector<cv::Vec2d> obs(views);
    for (int k = 0; k < n; ++k) {
        for (size_t v = 0; v < views; ++v)
            obs[v] = cv::Vec2d(xs[v].at<double>(0, k), xs[v].at<double>(1, k));
        const cv::Vec3d X = views == 2 ? triangulateDLT(obs[0], obs[1], Ps[0], Ps[1])
                                       : triangulateNView(obs, Ps);
        for (int r = 0; r < 3; ++r) points3d.at<double>(r, k) = X[r];
    }
}

}  // namespace mcslam::geom

#pragma once

// Linear multi-view triangulation and homogeneous-coordinate helpers.
//
// Drop-in replacement for the parts of OpenCV's contrib `sfm` module that
// MC-SLAM used (that module is not shipped by most distro OpenCV packages).
// The algorithms match cv::sfm: standard DLT for two views and the
// [P_i | -x_i] null-space formulation (Hartley & Zisserman) for N > 2 views.

#include <vector>

#include <opencv2/core.hpp>

namespace mcslam::geom {

/// Columns of a (d+1) x N matrix are homogeneous points; returns d x N.
void homogeneousToEuclidean(const cv::Mat& src, cv::Mat& dst);
void homogeneousToEuclidean(const cv::Mat& src, cv::Vec3d& dst);

/// Columns of a d x N matrix are points; returns (d+1) x N with a row of ones.
void euclideanToHomogeneous(const cv::Mat& src, cv::Mat& dst);

/// Triangulate N points observed in V views.
/// @param points2d    V matrices, each 2 x N (CV_64F), the observations per view.
/// @param projections V matrices, each 3 x 4 (CV_64F), the camera projection matrices.
/// @param points3d    Output 3 x N (CV_64F).
void triangulatePoints(const std::vector<cv::Mat>& points2d,
                       const std::vector<cv::Mat>& projections,
                       cv::Mat& points3d);

/// Convenience overload for typed matrices (e.g. std::vector<cv::Mat_<double>>).
template <typename T>
void triangulatePoints(const std::vector<cv::Mat_<T>>& points2d,
                       const std::vector<cv::Mat>& projections,
                       cv::Mat& points3d) {
    triangulatePoints(std::vector<cv::Mat>(points2d.begin(), points2d.end()), projections, points3d);
}

}  // namespace mcslam::geom

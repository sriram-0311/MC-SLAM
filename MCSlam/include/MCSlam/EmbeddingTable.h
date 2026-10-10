#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

namespace mcslam {

/// Per-frame global image descriptors computed offline (tools/place_recognition/embed_sequence.py),
/// looked up by image timestamp. File format: int32 N, int32 D, then N records of
/// (float64 timestamp [s], float32[D] descriptor), sorted by timestamp.
class EmbeddingTable {
public:
    /// Throws std::runtime_error if the file cannot be read.
    explicit EmbeddingTable(const std::string& path);

    int dim() const { return dim_; }
    size_t size() const { return times_.size(); }

    /// Descriptor of the frame closest to t, or nullptr if none is within tolerance [s].
    const float* lookup(double t, double tolerance = 1e-3) const;

private:
    int dim_ = 0;
    std::vector<double> times_;
    std::vector<float> data_;
};

}  // namespace mcslam

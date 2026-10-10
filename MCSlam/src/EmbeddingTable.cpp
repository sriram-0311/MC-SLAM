#include "MCSlam/EmbeddingTable.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>

namespace mcslam {

EmbeddingTable::EmbeddingTable(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in)
        throw std::runtime_error("cannot open embedding file " + path);
    int32_t n = 0, d = 0;
    in.read(reinterpret_cast<char*>(&n), sizeof(n));
    in.read(reinterpret_cast<char*>(&d), sizeof(d));
    if (!in || n <= 0 || d <= 0)
        throw std::runtime_error("bad embedding file header in " + path);
    dim_ = d;
    times_.resize(n);
    data_.resize(static_cast<size_t>(n) * d);
    for (int32_t i = 0; i < n; ++i) {
        in.read(reinterpret_cast<char*>(&times_[i]), sizeof(double));
        in.read(reinterpret_cast<char*>(&data_[static_cast<size_t>(i) * d]), sizeof(float) * d);
    }
    if (!in)
        throw std::runtime_error("truncated embedding file " + path);
    if (!std::is_sorted(times_.begin(), times_.end()))
        throw std::runtime_error("embedding file not sorted by time: " + path);
}

const float* EmbeddingTable::lookup(double t, double tolerance) const {
    const auto it = std::lower_bound(times_.begin(), times_.end(), t);
    size_t best = times_.size();
    double best_dt = tolerance;
    for (auto c : {it, it == times_.begin() ? it : it - 1}) {
        if (c == times_.end())
            continue;
        const double dt = std::abs(*c - t);
        if (dt <= best_dt) {
            best_dt = dt;
            best = static_cast<size_t>(c - times_.begin());
        }
    }
    return best == times_.size() ? nullptr : &data_[best * dim_];
}

}  // namespace mcslam

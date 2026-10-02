#pragma once

#include <chrono>
#include <cstdint>
#include <sstream>
#include <string>

namespace mcslam {

template <typename TimePoint>
int64_t usBetween(TimePoint start, TimePoint stop) {
    return std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();
}

/// Wall-clock cost of each pipeline stage for one input frame (microseconds; 0 = stage not run).
struct FrameTiming {
    double timestamp = 0.0;    // frame time [s]
    int64_t read_us = 0;       // image decode + IMU slicing
    int64_t extract_us = 0;    // ORB extraction + intra-rig matching (FrontEnd::processFrame)
    int64_t track_us = 0;      // frame-to-map tracking + keyframe decision (FrontEnd::trackFrame)
    int64_t graph_us = 0;      // factor-graph construction (Backend::SmartFactor_backend)
    int64_t optimize_us = 0;   // iSAM2 / LM update (Backend::optimizePosesLandmarks)
    int64_t update_us = 0;     // write-back of optimized variables to the map
    int64_t total_us = 0;      // whole frame, including read
    bool keyframe = false;
    bool loop_closure = false;

    static std::string csvHeader() {
        return "timestamp,read_us,extract_us,track_us,graph_us,optimize_us,update_us,total_us,keyframe,loop_closure";
    }

    std::string csvRow() const {
        std::ostringstream os;
        os.precision(9);
        os << std::fixed << timestamp << ',' << read_us << ',' << extract_us << ',' << track_us << ','
           << graph_us << ',' << optimize_us << ',' << update_us << ',' << total_us << ','
           << keyframe << ',' << loop_closure;
        return os.str();
    }
};

}  // namespace mcslam

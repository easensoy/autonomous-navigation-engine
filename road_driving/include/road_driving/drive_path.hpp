#pragma once
// The centreline the rear axle follows: the geometry of a sequence of route elements joined end to
// end, with one arc-length coordinate s.
#include "road_driving/lane_network.hpp"

#include <cstddef>
#include <vector>

namespace road {

struct PathSample {
    double x = 0.0, y = 0.0, yaw = 0.0, kappa = 0.0, s = 0.0;
    int element = -1;
    double speedLimit = 0.0;
};

class DrivePath {
public:
    DrivePath() = default;
    DrivePath(const LaneNetwork& net, const std::vector<int>& elements);

    bool empty() const { return samples_.empty(); }
    double length() const { return samples_.empty() ? 0.0 : samples_.back().s; }
    const std::vector<int>& elements() const { return elements_; }
    const std::vector<PathSample>& samples() const { return samples_; }
    double elementStart(std::size_t i) const { return starts_.at(i); }
    std::size_t elementIndexAt(double s) const;  // index into elements()
    int indexOf(int element) const;              // first index of an element id, -1 if absent
    std::size_t sampleIndexAt(double s) const;   // last sample with s_i <= s
    PathSample at(double s) const;               // interpolated; s is clamped to the path

    struct Projection {
        bool ok = false;
        double s = 0.0;
        double lateral = 0.0;  // positive left of the direction of travel
        double distance = 0.0;
    };
    // Nearest point within [sHint - back, sHint + ahead].
    Projection project(double x, double y, double sHint, double back = 5.0, double ahead = 25.0) const;

private:
    std::vector<int> elements_;
    std::vector<double> starts_;
    std::vector<PathSample> samples_;
};

}  // namespace road

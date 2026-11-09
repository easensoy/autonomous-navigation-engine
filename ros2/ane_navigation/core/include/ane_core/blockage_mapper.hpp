#pragma once
// Sensor-derived blockages: lidar returns that the static map does not explain are mapped onto
// roadmap nodes. A node is blocked when, in `consecutive_scans` scans in a row, at least `min_points`
// unexplained returns lie within `blocking_radius` of it. Blocked nodes stay latched until cleared.
//
// Why node blocking is enough for this roadmap: nodes are at most s apart along every edge, so a
// point within w of an edge's centre line lies within sqrt((s/2)^2 + w^2) of one of its end nodes.
// With s = 0.5 m and w = circumscribed radius 0.272 m + allowed cross-track 0.20 m = 0.472 m that
// bound is 0.534 m; blocking_radius is set to 0.55 m, so any return inside the corridor the robot may
// sweep along an edge blocks at least one end node of that edge.
#include "ane_core/roadmap.hpp"

#include <set>
#include <unordered_map>
#include <vector>

namespace ane {

struct BlockageMapperConfig {
    double blocking_radius = 0.55;   // m
    double static_tolerance = 0.10;  // m: returns this close to a static obstacle are explained by it
    int min_points = 3;              // unexplained returns near a node, per scan
    int consecutive_scans = 2;       // scans in a row that must meet min_points
};

struct ScanBlockages {
    std::vector<int> newly_blocked;  // nodes that met the rule for the first time in this scan
    std::vector<Point2> unexplained; // returns not explained by the static map, map frame
    std::size_t valid_points = 0;
};

class BlockageMapper {
public:
    BlockageMapper(const Roadmap& roadmap, BlockageMapperConfig config);
    ScanBlockages processScan(const std::vector<Point2>& mapPoints);
    bool explainedByStaticMap(const Point2& p) const;
    const std::set<int>& blockedNodes() const { return blocked_; }
    void clear(const std::vector<int>& nodes);
    void clearAll();
    const BlockageMapperConfig& config() const { return config_; }

private:
    const Roadmap& roadmap_;
    BlockageMapperConfig config_;
    std::vector<int> ids_;
    std::vector<Point2> positions_;
    std::unordered_map<int, int> streak_;
    std::set<int> blocked_;
};

// Converts a planar scan taken at sensorPose (map frame) into map-frame points. Ranges that are not
// finite or lie outside [rangeMin, rangeMax) are skipped: a return at the maximum range means that
// nothing was seen along that beam.
std::vector<Point2> scanToMapPoints(const Pose2& sensorPose, double angleMin, double angleIncrement,
                                    const std::vector<float>& ranges, double rangeMin, double rangeMax);

}  // namespace ane

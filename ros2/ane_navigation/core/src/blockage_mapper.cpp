#include "ane_core/blockage_mapper.hpp"

#include <cmath>
#include <stdexcept>

namespace ane {

BlockageMapper::BlockageMapper(const Roadmap& roadmap, BlockageMapperConfig config)
    : roadmap_(roadmap), config_(config) {
    if (!(config_.blocking_radius > 0.0) || config_.static_tolerance < 0.0 || config_.min_points < 1 ||
        config_.consecutive_scans < 1) {
        throw std::invalid_argument("blockage mapper: radius must be positive and the counts at least 1");
    }
    ids_ = roadmap_.nodeIds();
    for (int id : ids_) {
        positions_.push_back(roadmap_.nodePosition(id));
    }
}

bool BlockageMapper::explainedByStaticMap(const Point2& p) const {
    const Rect& hall = roadmap_.hall();
    if (p.x <= hall.min_x + config_.static_tolerance || p.x >= hall.max_x - config_.static_tolerance ||
        p.y <= hall.min_y + config_.static_tolerance || p.y >= hall.max_y - config_.static_tolerance) {
        return true;  // on or beyond the walls
    }
    for (const StaticObstacle& obstacle : roadmap_.obstacles()) {
        if (distanceToRect(p, obstacle.rect) <= config_.static_tolerance) {
            return true;
        }
    }
    return false;
}

ScanBlockages BlockageMapper::processScan(const std::vector<Point2>& mapPoints) {
    ScanBlockages result;
    result.valid_points = mapPoints.size();
    std::vector<int> hits(ids_.size(), 0);
    for (const Point2& p : mapPoints) {
        if (explainedByStaticMap(p)) {
            continue;
        }
        result.unexplained.push_back(p);
        for (std::size_t i = 0; i < ids_.size(); ++i) {
            if (distance(p, positions_[i]) <= config_.blocking_radius) {
                ++hits[i];
            }
        }
    }
    for (std::size_t i = 0; i < ids_.size(); ++i) {
        const int id = ids_[i];
        if (hits[i] >= config_.min_points) {
            const int streak = ++streak_[id];
            if (streak >= config_.consecutive_scans && blocked_.insert(id).second) {
                result.newly_blocked.push_back(id);
            }
        } else {
            streak_.erase(id);
        }
    }
    return result;
}

void BlockageMapper::clear(const std::vector<int>& nodes) {
    for (int id : nodes) {
        blocked_.erase(id);
        streak_.erase(id);
    }
}

void BlockageMapper::clearAll() {
    blocked_.clear();
    streak_.clear();
}

std::vector<Point2> scanToMapPoints(const Pose2& sensorPose, double angleMin, double angleIncrement,
                                    const std::vector<float>& ranges, double rangeMin, double rangeMax) {
    std::vector<Point2> points;
    points.reserve(ranges.size());
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        const double r = ranges[i];
        if (!std::isfinite(r) || r < rangeMin || r >= rangeMax) {
            continue;
        }
        const double angle = sensorPose.yaw + angleMin + angleIncrement * static_cast<double>(i);
        points.push_back({sensorPose.x + r * std::cos(angle), sensorPose.y + r * std::sin(angle)});
    }
    return points;
}

}  // namespace ane

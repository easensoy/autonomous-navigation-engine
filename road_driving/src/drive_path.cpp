#include "road_driving/drive_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace road {

DrivePath::DrivePath(const LaneNetwork& net, const std::vector<int>& elements) : elements_(elements) {
    double offset = 0.0;
    for (size_t i = 0; i < elements.size(); ++i) {
        const Element& e = net.elements.at(static_cast<size_t>(elements[i]));
        starts_.push_back(offset);
        for (size_t k = (i == 0 ? 0 : 1); k < e.pts.size(); ++k) {
            const PathPoint& p = e.pts[k];
            samples_.push_back({p.x, p.y, p.yaw, p.kappa, offset + p.s, e.id, e.speedLimit});
        }
        offset += e.length;
    }
}

std::size_t DrivePath::elementIndexAt(double s) const {
    if (starts_.empty()) {
        return 0;
    }
    const auto it = std::upper_bound(starts_.begin(), starts_.end(), s);
    return it == starts_.begin() ? 0 : static_cast<std::size_t>(it - starts_.begin() - 1);
}

int DrivePath::indexOf(int element) const {
    for (size_t i = 0; i < elements_.size(); ++i) {
        if (elements_[i] == element) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

std::size_t DrivePath::sampleIndexAt(double s) const {
    if (samples_.empty()) {
        return 0;
    }
    const auto it = std::upper_bound(samples_.begin(), samples_.end(), s,
                                     [](double v, const PathSample& p) { return v < p.s; });
    return it == samples_.begin() ? 0 : static_cast<std::size_t>(it - samples_.begin() - 1);
}

PathSample DrivePath::at(double s) const {
    if (samples_.empty()) {
        return {};
    }
    s = std::min(std::max(s, 0.0), length());
    const size_t i = std::min(sampleIndexAt(s), samples_.size() - 1);
    if (i + 1 >= samples_.size()) {
        return samples_.back();
    }
    const PathSample& a = samples_[i];
    const PathSample& b = samples_[i + 1];
    const double t = b.s > a.s ? (s - a.s) / (b.s - a.s) : 0.0;
    PathSample r = a;
    r.x = a.x + (b.x - a.x) * t;
    r.y = a.y + (b.y - a.y) * t;
    r.yaw = wrapAngle(a.yaw + wrapAngle(b.yaw - a.yaw) * t);
    r.kappa = a.kappa + (b.kappa - a.kappa) * t;
    r.s = s;
    return r;
}

DrivePath::Projection DrivePath::project(double x, double y, double sHint, double back, double ahead) const {
    Projection best;
    if (samples_.size() < 2) {
        return best;
    }
    const size_t i0 = sampleIndexAt(sHint - back);
    const size_t i1 = std::min(samples_.size() - 1, sampleIndexAt(sHint + ahead) + 1);
    double bestD = std::numeric_limits<double>::infinity();
    for (size_t i = i0; i < i1; ++i) {
        const PathSample& a = samples_[i];
        const PathSample& b = samples_[i + 1];
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double len2 = dx * dx + dy * dy;
        double t = len2 > 0.0 ? ((x - a.x) * dx + (y - a.y) * dy) / len2 : 0.0;
        t = std::min(1.0, std::max(0.0, t));
        const double qx = a.x + dx * t, qy = a.y + dy * t;
        const double d = std::hypot(x - qx, y - qy);
        if (d < bestD) {
            bestD = d;
            best.ok = true;
            best.s = a.s + (b.s - a.s) * t;
            best.distance = d;
            const double len = std::sqrt(len2);
            best.lateral = len > 0.0 ? (dx * (y - qy) - dy * (x - qx)) / len : 0.0;
        }
    }
    return best;
}

}  // namespace road

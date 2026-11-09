#include "ane_core/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ane {

namespace {
constexpr double kPi = 3.14159265358979323846;

double cross2(const Point2& o, const Point2& a, const Point2& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

bool onSegment(const Point2& a, const Point2& b, const Point2& p) {
    return std::min(a.x, b.x) - 1e-12 <= p.x && p.x <= std::max(a.x, b.x) + 1e-12 &&
           std::min(a.y, b.y) - 1e-12 <= p.y && p.y <= std::max(a.y, b.y) + 1e-12;
}

bool segmentsTouch(const Point2& a, const Point2& b, const Point2& c, const Point2& d) {
    const double d1 = cross2(c, d, a);
    const double d2 = cross2(c, d, b);
    const double d3 = cross2(a, b, c);
    const double d4 = cross2(a, b, d);
    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
        return true;
    }
    return (d1 == 0 && onSegment(c, d, a)) || (d2 == 0 && onSegment(c, d, b)) ||
           (d3 == 0 && onSegment(a, b, c)) || (d4 == 0 && onSegment(a, b, d));
}
}  // namespace

bool Rect::contains(const Point2& p) const {
    return p.x >= min_x && p.x <= max_x && p.y >= min_y && p.y <= max_y;
}

double wrapAngle(double angle) {
    return std::remainder(angle, 2.0 * kPi);
}

double distance(const Point2& a, const Point2& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

double distanceToRect(const Point2& p, const Rect& r) {
    const double dx = std::max({r.min_x - p.x, 0.0, p.x - r.max_x});
    const double dy = std::max({r.min_y - p.y, 0.0, p.y - r.max_y});
    return std::hypot(dx, dy);
}

double distanceToSegment(const Point2& p, const Point2& a, const Point2& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    if (length2 == 0.0) {
        return distance(p, a);
    }
    const double t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length2, 0.0, 1.0);
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

double segmentRectDistance(const Point2& a, const Point2& b, const Rect& r) {
    if (r.contains(a) || r.contains(b)) {
        return 0.0;
    }
    const Point2 corners[4] = {{r.min_x, r.min_y}, {r.max_x, r.min_y}, {r.max_x, r.max_y}, {r.min_x, r.max_y}};
    for (int i = 0; i < 4; ++i) {
        if (segmentsTouch(a, b, corners[i], corners[(i + 1) % 4])) {
            return 0.0;
        }
    }
    // Disjoint convex shapes: the closest pair involves an end point of the segment or a corner.
    double best = std::min(distanceToRect(a, r), distanceToRect(b, r));
    for (const Point2& corner : corners) {
        best = std::min(best, distanceToSegment(corner, a, b));
    }
    return best;
}

LineCoordinates lineCoordinates(const Point2& p, const Point2& a, const Point2& b) {
    LineCoordinates result;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    result.length = std::hypot(dx, dy);
    if (result.length == 0.0) {
        result.along = 0.0;
        result.cross = distance(p, a);
        return result;
    }
    const double ux = dx / result.length;
    const double uy = dy / result.length;
    const double px = p.x - a.x;
    const double py = p.y - a.y;
    result.along = px * ux + py * uy;
    result.cross = ux * py - uy * px;
    return result;
}

double rayRectDistance(const Point2& origin, double dx, double dy, const Rect& r) {
    if (r.contains(origin)) {
        return 0.0;
    }
    double tNear = -std::numeric_limits<double>::infinity();
    double tFar = std::numeric_limits<double>::infinity();
    const double o[2] = {origin.x, origin.y};
    const double d[2] = {dx, dy};
    const double lo[2] = {r.min_x, r.min_y};
    const double hi[2] = {r.max_x, r.max_y};
    for (int axis = 0; axis < 2; ++axis) {
        if (std::abs(d[axis]) < 1e-15) {
            if (o[axis] < lo[axis] || o[axis] > hi[axis]) {
                return -1.0;
            }
            continue;
        }
        double t1 = (lo[axis] - o[axis]) / d[axis];
        double t2 = (hi[axis] - o[axis]) / d[axis];
        if (t1 > t2) {
            std::swap(t1, t2);
        }
        tNear = std::max(tNear, t1);
        tFar = std::min(tFar, t2);
        if (tNear > tFar || tFar < 0.0) {
            return -1.0;
        }
    }
    return tNear >= 0.0 ? tNear : -1.0;
}

}  // namespace ane

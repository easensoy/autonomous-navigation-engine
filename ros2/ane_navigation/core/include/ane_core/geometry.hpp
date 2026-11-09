#pragma once
// Planar geometry used by the adapter core: map-frame points, poses and axis-aligned rectangles.
// Lengths are metres and angles radians, counter-clockwise from +x (REP 103).

namespace ane {

struct Point2 {
    double x = 0.0;
    double y = 0.0;
};

struct Pose2 {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    Point2 position() const { return {x, y}; }
};

struct Rect {
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    bool contains(const Point2& p) const;
};

double wrapAngle(double angle);  // into [-pi, pi]
double distance(const Point2& a, const Point2& b);
double distanceToRect(const Point2& p, const Rect& r);  // 0 inside or on the boundary
double distanceToSegment(const Point2& p, const Point2& a, const Point2& b);
double segmentRectDistance(const Point2& a, const Point2& b, const Rect& r);  // 0 when they touch

// Position of p relative to the directed line a -> b: distance along the line from a, signed distance
// across it (positive to the left of the direction of travel), and the length |b - a|.
struct LineCoordinates {
    double along = 0.0;
    double cross = 0.0;
    double length = 0.0;
};
LineCoordinates lineCoordinates(const Point2& p, const Point2& a, const Point2& b);

// Distance from origin along the unit direction (dx, dy) to the boundary of r, or -1 when the ray
// misses it. An origin inside r returns 0.
double rayRectDistance(const Point2& origin, double dx, double dy, const Rect& r);

}  // namespace ane

#pragma once
// Test fixture for the closed-loop adapter test: an ideal differential-drive robot (unicycle
// kinematics) whose velocities follow the command under the same velocity and acceleration limits as
// the Gazebo DiffDrive configuration, and a planar lidar ray-cast against rectangles.
//
// There is no physics, wheel slip, sensor noise or latency here. This fixture exercises the adapter's
// logic in closed loop; its outcomes are test results, not simulation results.
#include "ane_core/geometry.hpp"
#include "ane_core/segment_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace ane::test {

struct PlantLimits {  // match ane_description/urdf/ane_amr.gazebo.xacro (DiffDrive)
    double max_linear = 0.6;
    double max_angular = 1.5;
    double max_linear_accel = 1.0;
    double max_angular_accel = 3.0;
};

class UnicyclePlant {
public:
    UnicyclePlant(Pose2 start, PlantLimits limits) : pose_(start), limits_(limits) {}

    void step(const VelocityCommand& command, double dt) {
        const double v = std::clamp(command.linear, -limits_.max_linear, limits_.max_linear);
        const double w = std::clamp(command.angular, -limits_.max_angular, limits_.max_angular);
        v_ += std::clamp(v - v_, -limits_.max_linear_accel * dt, limits_.max_linear_accel * dt);
        w_ += std::clamp(w - w_, -limits_.max_angular_accel * dt, limits_.max_angular_accel * dt);
        if (std::abs(w_) < 1e-9) {
            pose_.x += v_ * std::cos(pose_.yaw) * dt;
            pose_.y += v_ * std::sin(pose_.yaw) * dt;
        } else {
            const double yaw2 = pose_.yaw + w_ * dt;
            pose_.x += v_ / w_ * (std::sin(yaw2) - std::sin(pose_.yaw));
            pose_.y -= v_ / w_ * (std::cos(yaw2) - std::cos(pose_.yaw));
            pose_.yaw = wrapAngle(yaw2);
        }
    }

    const Pose2& pose() const { return pose_; }
    double linear() const { return v_; }
    double angular() const { return w_; }

private:
    Pose2 pose_;
    PlantLimits limits_;
    double v_ = 0.0;
    double w_ = 0.0;
};

// Ranges of `samples` beams starting at angleMin (sensor frame); beams that hit nothing within
// rangeMax return +infinity, as Gazebo's gpu_lidar does.
inline std::vector<float> castScan(const Pose2& sensor, int samples, double angleMin, double increment,
                                   double rangeMax, const std::vector<Rect>& rects) {
    std::vector<float> ranges(static_cast<std::size_t>(samples), std::numeric_limits<float>::infinity());
    for (int i = 0; i < samples; ++i) {
        const double angle = sensor.yaw + angleMin + increment * i;
        const double dx = std::cos(angle);
        const double dy = std::sin(angle);
        double nearest = rangeMax;
        for (const Rect& r : rects) {
            const double d = rayRectDistance(sensor.position(), dx, dy, r);
            if (d >= 0.0 && d < nearest) {
                nearest = d;
            }
        }
        if (nearest < rangeMax) {
            ranges[static_cast<std::size_t>(i)] = static_cast<float>(nearest);
        }
    }
    return ranges;
}

// Smallest distance between the robot footprint (a length x width rectangle centred on the pose) and
// any rectangle, sampled every 1 cm along the footprint outline; 0 when an outline sample lies inside a
// rectangle or a rectangle corner lies inside the footprint.
inline double footprintClearance(const Pose2& pose, double length, double width, const std::vector<Rect>& rects) {
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    auto toWorld = [&](double lx, double ly) { return Point2{pose.x + c * lx - s * ly, pose.y + s * lx + c * ly}; };
    std::vector<Point2> outline;
    const double hl = length / 2.0;
    const double hw = width / 2.0;
    for (double t = -hl; t <= hl + 1e-9; t += 0.01) {
        outline.push_back(toWorld(t, hw));
        outline.push_back(toWorld(t, -hw));
    }
    for (double t = -hw; t <= hw + 1e-9; t += 0.01) {
        outline.push_back(toWorld(hl, t));
        outline.push_back(toWorld(-hl, t));
    }
    double best = std::numeric_limits<double>::infinity();
    for (const Rect& r : rects) {
        const Point2 corners[4] = {{r.min_x, r.min_y}, {r.max_x, r.min_y}, {r.max_x, r.max_y}, {r.min_x, r.max_y}};
        for (const Point2& corner : corners) {
            const double lx = c * (corner.x - pose.x) + s * (corner.y - pose.y);
            const double ly = -s * (corner.x - pose.x) + c * (corner.y - pose.y);
            if (std::abs(lx) <= hl && std::abs(ly) <= hw) {
                return 0.0;
            }
        }
        for (const Point2& p : outline) {
            best = std::min(best, distanceToRect(p, r));
        }
    }
    return best;
}

}  // namespace ane::test

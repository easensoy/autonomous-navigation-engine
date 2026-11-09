#pragma once
// Drives a differential-drive robot along one straight leg of the route.
//
//   ROTATE  the heading error to the leg direction exceeds rotate_enter: turn in place (v = 0) until it
//           is below rotate_exit. A robot that is still moving first brakes (BRAKE).
//   DRIVE   pure pursuit on the leg's line, look-ahead `lookahead`, with the speed planned to stop at the
//           next stop point: v <= sqrt(2 a d), where d is the remaining distance to the stop point.
//
// The robot therefore drives along edges and turns only on nodes; it never cuts a corner between two
// edges. Every command is saturated to the limits and its change per step is bounded by the
// acceleration limits, so |v| <= max_linear, |w| <= max_angular, |dv/dt| <= max_linear_decel and
// |dw/dt| <= max_angular_accel hold for every output, including brake().
#include "ane_core/geometry.hpp"

namespace ane {

struct ControllerLimits {
    double max_linear = 0.5;          // m/s
    double max_angular = 1.0;         // rad/s
    double max_linear_accel = 0.5;    // m/s^2: speeding up, and the planned slow-down before a stop point
    double max_linear_decel = 1.0;    // m/s^2: braking
    double max_angular_accel = 2.0;   // rad/s^2
};

struct ControllerConfig {
    ControllerLimits limits;
    double lookahead = 0.40;        // m
    double rotate_enter = 0.30;     // rad
    double rotate_exit = 0.04;      // rad
    double rotate_gain = 1.5;       // 1/s
    double min_rotate_rate = 0.15;  // rad/s
    double creep_speed = 0.04;      // m/s: lowest speed while short of the stop point
    double stop_tolerance = 0.04;   // m: commanded speed is zero within this distance of the stop point
};

enum class ControlMode { IDLE, ROTATE, DRIVE, BRAKE };
const char* controlModeName(ControlMode mode);

struct VelocityCommand {
    double linear = 0.0;   // m/s
    double angular = 0.0;  // rad/s
};

// A leg from `start` to `end` (map frame). `stop_distance` is the distance along the route from `end`
// to the next stop point: 0 when `end` is itself a stop point (a turn or the goal), otherwise the
// length of the straight run that continues past `end`.
struct Leg {
    Point2 start;
    Point2 end;
    double stop_distance = 0.0;
};

class SegmentController {
public:
    explicit SegmentController(ControllerConfig config = {});

    VelocityCommand track(const Pose2& pose, const Leg& leg, double dt);
    VelocityCommand brake(double dt);
    void reset();  // forget the last command; use only when the robot has been commanded to zero

    ControlMode mode() const { return mode_; }
    const VelocityCommand& lastCommand() const { return last_; }
    double crossTrack() const { return cross_track_; }
    double headingError() const { return heading_error_; }
    double distanceToStop() const { return distance_to_stop_; }
    const ControllerConfig& config() const { return config_; }

private:
    VelocityCommand limit(const VelocityCommand& desired, double dt);

    ControllerConfig config_;
    ControlMode mode_ = ControlMode::IDLE;
    VelocityCommand last_{};
    double cross_track_ = 0.0;
    double heading_error_ = 0.0;
    double distance_to_stop_ = 0.0;
};

}  // namespace ane

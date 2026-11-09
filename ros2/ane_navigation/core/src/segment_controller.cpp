#include "ane_core/segment_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ane {

const char* controlModeName(ControlMode mode) {
    switch (mode) {
        case ControlMode::IDLE: return "IDLE";
        case ControlMode::ROTATE: return "ROTATE";
        case ControlMode::DRIVE: return "DRIVE";
        case ControlMode::BRAKE: return "BRAKE";
    }
    return "UNKNOWN";
}

SegmentController::SegmentController(ControllerConfig config) : config_(config) {
    const ControllerLimits& l = config_.limits;
    if (!(l.max_linear > 0 && l.max_angular > 0 && l.max_linear_accel > 0 && l.max_linear_decel > 0 &&
          l.max_angular_accel > 0 && config_.lookahead > 0 && config_.rotate_exit > 0 &&
          config_.rotate_enter > config_.rotate_exit && config_.stop_tolerance >= 0)) {
        throw std::invalid_argument("segment controller: limits and thresholds must be positive, rotate_enter > rotate_exit");
    }
}

void SegmentController::reset() {
    last_ = {};
    mode_ = ControlMode::IDLE;
}

VelocityCommand SegmentController::limit(const VelocityCommand& desired, double dt) {
    const ControllerLimits& l = config_.limits;
    const double target = std::clamp(desired.linear, -l.max_linear, l.max_linear);
    const bool slowing = std::abs(target) < std::abs(last_.linear) || target * last_.linear < 0.0;
    const double step = (slowing ? l.max_linear_decel : l.max_linear_accel) * dt;
    VelocityCommand out;
    out.linear = last_.linear + std::clamp(target - last_.linear, -step, step);
    const double targetAngular = std::clamp(desired.angular, -l.max_angular, l.max_angular);
    const double angularStep = l.max_angular_accel * dt;
    out.angular = last_.angular + std::clamp(targetAngular - last_.angular, -angularStep, angularStep);
    out.linear = std::clamp(out.linear, -l.max_linear, l.max_linear);
    out.angular = std::clamp(out.angular, -l.max_angular, l.max_angular);
    last_ = out;
    return out;
}

VelocityCommand SegmentController::brake(double dt) {
    return limit({0.0, 0.0}, dt);
}

VelocityCommand SegmentController::track(const Pose2& pose, const Leg& leg, double dt) {
    const LineCoordinates lc = lineCoordinates(pose.position(), leg.start, leg.end);
    const double direction = std::atan2(leg.end.y - leg.start.y, leg.end.x - leg.start.x);
    heading_error_ = wrapAngle(direction - pose.yaw);
    cross_track_ = lc.cross;
    distance_to_stop_ = std::max(0.0, lc.length - lc.along) + leg.stop_distance;

    if ((mode_ == ControlMode::IDLE || mode_ == ControlMode::DRIVE) && std::abs(heading_error_) > config_.rotate_enter) {
        mode_ = (std::abs(last_.linear) > 1e-3) ? ControlMode::BRAKE : ControlMode::ROTATE;
    }
    if (mode_ == ControlMode::BRAKE) {
        const VelocityCommand out = limit({0.0, 0.0}, dt);
        if (std::abs(out.linear) < 1e-3 && std::abs(out.angular) < 1e-3) {
            mode_ = ControlMode::ROTATE;
        }
        return out;
    }
    if (mode_ == ControlMode::ROTATE) {
        if (std::abs(heading_error_) > config_.rotate_exit) {
            double rate = config_.rotate_gain * heading_error_;
            if (std::abs(rate) < config_.min_rotate_rate) {
                rate = std::copysign(config_.min_rotate_rate, heading_error_);
            }
            return limit({0.0, rate}, dt);
        }
        mode_ = ControlMode::DRIVE;
    }
    if (mode_ == ControlMode::IDLE) {
        mode_ = ControlMode::DRIVE;
    }

    // DRIVE: speed profile towards the stop point, pure pursuit on the leg line.
    if (distance_to_stop_ <= config_.stop_tolerance) {
        return limit({0.0, 0.0}, dt);
    }
    const ControllerLimits& l = config_.limits;
    double speed = std::min(l.max_linear,
                            std::sqrt(2.0 * l.max_linear_accel * (distance_to_stop_ - config_.stop_tolerance)));
    speed *= std::clamp(1.0 - std::abs(heading_error_) / config_.rotate_enter, 0.3, 1.0);
    speed = std::max(speed, config_.creep_speed);

    const double ux = (lc.length > 0.0) ? (leg.end.x - leg.start.x) / lc.length : std::cos(direction);
    const double uy = (lc.length > 0.0) ? (leg.end.y - leg.start.y) / lc.length : std::sin(direction);
    const double reach = std::min(lc.along + config_.lookahead, lc.length + leg.stop_distance);
    const Point2 target{leg.start.x + ux * reach, leg.start.y + uy * reach};
    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double lookDistance = std::max(std::hypot(dx, dy), 1e-3);
    const double alpha = wrapAngle(std::atan2(dy, dx) - pose.yaw);
    const double angular = 2.0 * speed * std::sin(alpha) / lookDistance;
    return limit({speed, angular}, dt);
}

}  // namespace ane

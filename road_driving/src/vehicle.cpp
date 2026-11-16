#include "road_driving/vehicle.hpp"

#include "road_driving/lane_network.hpp"

#include <algorithm>
#include <cmath>

namespace road {

Vehicle::Vehicle(const VehicleParams& params, double dt, const VehicleState& initial)
    : p_(params), dt_(dt), state_(initial) {
    const long n = std::lround(p_.commandDelay / dt_);
    delay_.assign(static_cast<size_t>(std::max(0L, n)), Command{initial.steer, 0.0});
}

void Vehicle::step(const Command& command) {
    Command applied = command;
    if (!delay_.empty()) {
        applied = delay_[head_];
        delay_[head_] = command;
        head_ = (head_ + 1) % delay_.size();
    }

    // Steering: first-order lag towards the delayed command, then rate and angle limits.
    const double target = std::min(p_.maxSteer, std::max(-p_.maxSteer, applied.steer));
    double rate = p_.steerLag > 0.0 ? (target - state_.steer) / p_.steerLag : (target - state_.steer) / dt_;
    rate = std::min(p_.maxSteerRate, std::max(-p_.maxSteerRate, rate));
    const double steer0 = state_.steer;
    state_.steer = std::min(p_.maxSteer, std::max(-p_.maxSteer, state_.steer + rate * dt_));
    lastSteerRate_ = (state_.steer - steer0) / dt_;

    // Longitudinal: first-order lag towards the delayed command within the physical limits.
    const double aTarget = std::min(p_.maxAccel, std::max(-p_.decelEmergency, applied.accel));
    if (p_.longLag > 0.0) {
        state_.accel += (aTarget - state_.accel) * std::min(1.0, dt_ / p_.longLag);
    } else {
        state_.accel = aTarget;
    }

    // Kinematic bicycle about the rear axle; the car stops rather than reverses.
    const double v0 = state_.v;
    double v1 = v0 + state_.accel * dt_;
    double ds;
    if (v1 < 0.0) {
        ds = state_.accel < 0.0 ? v0 * v0 / (-2.0 * state_.accel) : 0.0;
        v1 = 0.0;
    } else {
        ds = 0.5 * (v0 + v1) * dt_;
    }
    const double dyaw = ds * std::tan(state_.steer) / p_.wheelbase;
    state_.x += ds * std::cos(state_.yaw + 0.5 * dyaw);
    state_.y += ds * std::sin(state_.yaw + 0.5 * dyaw);
    state_.yaw = wrapAngle(state_.yaw + dyaw);
    state_.v = v1;
}

}  // namespace road

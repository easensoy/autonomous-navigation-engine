#include "road_driving/vehicle.hpp"

#include "road_driving/lane_network.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace road {

namespace {

// The Cybercab-inspired proxy. Sourced from Tesla's public Cybercab pages (FAQ and rider guide, read
// 2026-11-19): two seats, butterfly doors, no steering wheel or pedals, front and rear light bars, a rear trunk
// and no front trunk, ground clearance 5.8 in (0.147 m), step-in height 16.5 in, shoulder room 51.9 in (1.318 m).
// Research assumptions (no published figure): the body width (the shoulder room plus about 0.3 m a side), the
// wheelbase, the overhangs and length of a compact two-seat body, no exterior mirrors (the rider guide lists
// none among the exterior features), the roof GNSS antenna and the IMU above the cabin floor, the kerb scanner
// at the front bumper. Actuator limits are the I-PACE proxy's.
VehicleParams cybercabProxy() {
    VehicleParams v;
    v.id = "cybercab_proxy_v1";
    v.description = "kinematic bicycle (Cybercab-inspired proxy; dimensions are research assumptions)";
    v.wheelbase = 2.75;
    v.length = 4.20;
    v.width = 1.93;
    v.widthMirrors = 1.93;  // no exterior mirrors modelled
    v.frontOverhang = 0.80;
    v.rearOverhang = 0.65;
    v.gnssAntennaX = 1.30;
    v.gnssAntennaY = 0.0;
    v.imuX = 1.10;
    v.kerbScannerX = 3.55;  // wheelbase + front overhang
    v.height = 1.44;
    v.wheelRadius = 0.36;
    v.track = 1.62;
    return v;
}

}  // namespace

bool findVehicleProfile(const std::string& id, VehicleParams& out) {
    if (id == "ipace_proxy_v1") {
        out = VehicleParams{};
        return true;
    }
    if (id == "cybercab_proxy_v1") {
        out = cybercabProxy();
        return true;
    }
    return false;
}

VehicleParams vehicleProfile(const std::string& id) {
    VehicleParams vp;
    if (!findVehicleProfile(id, vp)) {
        throw std::invalid_argument("unknown vehicle profile '" + id + "' (known: ipace_proxy_v1, cybercab_proxy_v1)");
    }
    return vp;
}

std::vector<std::string> vehicleProfileNames() { return {"ipace_proxy_v1", "cybercab_proxy_v1"}; }

std::string vehicleProfileText(const VehicleParams& v) {
    char buf[640];
    std::snprintf(buf, sizeof buf,
                  "id=%s;wheelbase=%.3f;length=%.3f;width=%.3f;width_mirrors=%.3f;front_overhang=%.3f;rear_overhang=%.3f;max_steer=%.3f;"
                  "max_steer_rate=%.3f;steer_lag=%.3f;command_delay=%.3f;max_curvature=%.3f;accel_comfort=%.2f;decel_comfort=%.2f;decel_firm=%.2f;"
                  "decel_emergency=%.2f;max_accel=%.2f;lat_accel_limit=%.2f;long_lag=%.3f;gnss_antenna_x=%.3f;gnss_antenna_y=%.3f;imu_x=%.3f;"
                  "kerb_scanner_x=%.3f",
                  v.id, v.wheelbase, v.length, v.width, v.widthMirrors, v.frontOverhang, v.rearOverhang, v.maxSteer, v.maxSteerRate, v.steerLag,
                  v.commandDelay, v.maxCurvature, v.accelComfort, v.decelComfort, v.decelFirm, v.decelEmergency, v.maxAccel, v.latAccelLimit,
                  v.longLag, v.gnssAntennaX, v.gnssAntennaY, v.imuX, v.kerbScannerX);
    return buf;
}

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

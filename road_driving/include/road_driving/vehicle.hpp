#pragma once
// Fixed-step kinematic bicycle for an approximately I-PACE-sized car (reference point: rear-axle
// centre). Commands pass through a fixed delay; steering then follows a first-order lag with rate and
// angle limits; longitudinal acceleration follows a first-order lag within braking and drive limits.
// No reverse. Values are a proxy assembled for this study, not a model of any real vehicle.
#include <cstddef>
#include <vector>

namespace road {

struct VehicleParams {
    const char* id = "ipace_proxy_v1";
    double wheelbase = 2.990;      // m, published wheelbase
    double length = 4.682;         // m
    double width = 2.011;          // m, body
    double widthMirrors = 2.139;   // m, used for clearance
    double frontOverhang = 0.850;  // m, assumed split of the published 1.692 m total
    double rearOverhang = 0.842;   // m
    double maxSteer = 0.55;        // rad, road-wheel angle
    double maxSteerRate = 0.4;     // rad/s
    double steerLag = 0.20;        // s
    double commandDelay = 0.10;    // s, applies to steering and acceleration commands
    double maxCurvature = 0.20;    // 1/m, planning limit
    double accelComfort = 1.5;     // m/s^2
    double decelComfort = 2.0;     // m/s^2
    double decelFirm = 3.5;        // m/s^2
    double decelEmergency = 8.0;   // m/s^2, also the physical braking limit of the model
    double maxAccel = 3.0;         // m/s^2, physical drive limit of the model
    double latAccelLimit = 2.0;    // m/s^2, design limit
    double longLag = 0.30;         // s
    double frontOffset() const { return wheelbase + frontOverhang; }  // rear axle to front bumper
};

struct VehicleState {
    double x = 0.0, y = 0.0, yaw = 0.0;  // rear-axle centre, heading
    double v = 0.0;                      // m/s, never negative
    double steer = 0.0;                  // rad, actual road-wheel angle
    double accel = 0.0;                  // m/s^2, actual longitudinal acceleration
};

struct Command {
    double steer = 0.0;
    double accel = 0.0;
};

class Vehicle {
public:
    Vehicle(const VehicleParams& params, double dt, const VehicleState& initial);
    void step(const Command& command);  // advance one tick
    const VehicleState& state() const { return state_; }
    double lastSteerRate() const { return lastSteerRate_; }  // rad/s over the last tick

private:
    VehicleParams p_;
    double dt_;
    VehicleState state_;
    std::vector<Command> delay_;  // ring buffer of round(commandDelay / dt) commands
    std::size_t head_ = 0;
    double lastSteerRate_ = 0.0;
};

}  // namespace road

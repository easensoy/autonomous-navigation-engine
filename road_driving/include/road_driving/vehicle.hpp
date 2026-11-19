#pragma once
// Fixed-step kinematic bicycle (reference point: rear-axle centre). Commands pass through a fixed delay;
// steering then follows a first-order lag with rate and angle limits; longitudinal acceleration follows a
// first-order lag within braking and drive limits. No reverse.
//
// The vehicle is one of a small set of versioned profiles, each the one place its geometry, limits and sensor
// mounting are defined. Every consumer takes them from here: the planner's feasibility and the controller, the
// swept-body evaluator, the simulated sensors and the estimators' lever arms, and the 3D viewer (through the
// live scene message). Values are proxies assembled for this study, not models of any real vehicle:
// - "ipace_proxy_v1" (the default, and every result before November 2026): approximately I-PACE-sized, from
//   published dimensions.
// - "cybercab_proxy_v1": a Cybercab-inspired two-seat proxy. Tesla's public Cybercab pages give its ground
//   clearance, step-in height and cabin dimensions, not its length, width or wheelbase: those, the overhangs
//   and the sensor mounts are research assumptions, and the actuator limits are the I-PACE proxy's, so that a
//   comparison between the two profiles isolates geometry and sensor mounting (road_driving/README.md).
#include <cstddef>
#include <string>
#include <vector>

namespace road {

struct VehicleParams {
    const char* id = "ipace_proxy_v1";
    const char* description = "kinematic bicycle (approximate I-PACE dimensions)";  // the manifest's vehicle line
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
    // Sensor mounting, body frame (origin at the rear-axle centre, x forward, y left), m. The simulated sensors
    // are placed here and the estimators use the same values as installation data.
    double gnssAntennaX = 1.60, gnssAntennaY = 0.0;  // roof antenna
    double imuX = 1.40;                              // IMU, on the centre line
    double kerbScannerX = 3.84;                      // planar kerb scanner, front bumper centre at kerb height
    // Drawing only (the 3D viewer): body height, wheel radius and track. No simulated quantity depends on them.
    double height = 1.565, wheelRadius = 0.377, track = 1.64;
    double frontOffset() const { return wheelbase + frontOverhang; }  // rear axle to front bumper
};

// The profiles by id; false for an unknown one.
bool findVehicleProfile(const std::string& id, VehicleParams& out);
// The same, throwing std::invalid_argument for an unknown id.
VehicleParams vehicleProfile(const std::string& id);
std::vector<std::string> vehicleProfileNames();  // "ipace_proxy_v1", "cybercab_proxy_v1"
// Every simulated parameter on one line, "key=value;..." (manifests and benchmark records).
std::string vehicleProfileText(const VehicleParams& vp);

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

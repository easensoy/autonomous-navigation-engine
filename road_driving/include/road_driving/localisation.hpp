#pragma once
// Simulated sensors and state estimation for road_driving.
//
// Frames and units. The map frame is the network's (x east, y north, metres). The body frame has its
// origin at the rear-axle centre, x forward and y left. Angles are radians, anticlockwise from map x.
// Time is seconds since the start of the episode. Every measurement carries the time it is valid for
// (tMeas) and the time it reaches the estimator (tArrival >= tMeas).
//
// Separation. The world turns its ground truth into measurements (SensorSimulator). Estimators receive
// only those measurements, in arrival order, and the lane map (used once, for the initial heading of a
// car parked in a lane). Nothing derived from truth reaches an estimator any other way.
#include "road_driving/lane_network.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace road {

// ---- Measurements -------------------------------------------------------------------------------------
struct Measurement {
    enum Kind { Gnss = 0, Imu = 1, Wheel = 2, Steer = 3, Kerb = 4 };
    Kind kind = Imu;
    double tMeas = 0.0;
    double tArrival = 0.0;
    // Gnss: antenna x, y (map frame, m) and the receiver's reported 1-sigma (m).
    // Imu: specific force along body x and y (m/s^2) and yaw rate (rad/s) at the IMU.
    // Wheel: rear-axle speed from the wheel encoders (m/s). Steer: road-wheel angle (rad).
    // Kerb: a = bearing of the first beam and b = the step between beams (rad, body frame), c = the
    // scanner's position ahead of the rear axle (m); `ranges` holds one range per beam (m, -1: no return).
    double a = 0.0, b = 0.0, c = 0.0;
    std::vector<double> ranges;
};
const char* kindName(Measurement::Kind k);

// ---- Sensor configuration and conditions --------------------------------------------------------------
struct GnssConfig {
    double rate = 10.0;           // Hz
    double latency = 0.10;        // s from time of validity to arrival
    double latencyJitter = 0.02;  // s, uniform +-
    double sigma = 0.25;          // m per axis, white
    double corrSigma = 0.15;      // m per axis, first-order Gauss-Markov
    double corrTau = 60.0;        // s
    double dropProb = 0.02;       // fixes lost at random
    double antennaX = 1.60;       // m, body frame
    double antennaY = 0.0;
    double reportedSigma = 0.30;  // m, what the receiver reports (constant; it does not see multipath)
};
struct ImuConfig {
    double rate = 100.0;
    double leverX = 1.40, leverY = 0.0;  // m, IMU position in the body frame (axes aligned)
    double gyroNoise = 0.0015;           // rad/s, 1-sigma per sample
    double gyroBias = 0.0025;            // rad/s, initial magnitude (sign drawn per seed)
    double gyroBiasWalk = 2e-5;          // rad/s per sqrt(s)
    double accelNoise = 0.03;            // m/s^2, 1-sigma per sample
    double accelBias = 0.04;             // m/s^2, initial magnitude (sign drawn per seed)
    double accelBiasWalk = 1e-4;         // m/s^2 per sqrt(s)
};
struct WheelConfig {
    double rate = 50.0;
    double latency = 0.02;
    double noise = 0.02;        // m/s
    double scaleError = 0.01;   // tyre radius error, magnitude (sign drawn per seed)
    double quantum = 0.005;     // m/s, encoder resolution; reads exactly 0 at rest
};
struct SteerConfig {
    double rate = 50.0;
    double latency = 0.02;
    double noise = 0.002;   // rad
    double offset = 0.004;  // rad, magnitude (sign drawn per seed)
};
// A planar range scanner at kerb height on the front bumper, facing forward. A beam returns where it
// first meets the kerb (or an object standing in the carriageway); nothing else is modelled. The real
// kerb differs from the map by a smooth deviation (map error and kerb irregularity).
struct KerbConfig {
    double rate = 10.0;               // Hz
    double latency = 0.05;            // s, scan processing
    double mountX = 3.84;             // m ahead of the rear axle (front bumper centre)
    double firstBearing = -0.5 * 3.14159265358979323846;  // rad, body frame
    double bearingStep = 5.0 * 3.14159265358979323846 / 180.0;
    int beams = 37;                   // -90 to +90 degrees
    double minRange = 0.5, maxRange = 25.0;  // m
    double rangeNoise = 0.03;         // m, 1-sigma
    double dropout = 0.02;            // returns lost at random
    double kerbDeviation = 0.03;      // m RMS, real kerb against the map, varying along it
    double deviationWavelength = 25.0;  // m
};
// A stretch of the route, as fractions of its length; the world turns it into map-frame rectangles.
struct Span {
    double from = 0.0, to = 0.0;
    bool active() const { return to > from; }
};
struct SensorConditions {
    std::string name = "nominal";
    GnssConfig gnss;
    ImuConfig imu;
    WheelConfig wheel;
    SteerConfig steer;
    KerbConfig kerb;
    Span outage;                    // no GNSS fixes (urban canyon, underpass)
    Span multipath;                 // fixes shifted by a constant offset of this magnitude, extra noise
    double multipathBias = 0.0;     // m (direction drawn per seed)
    double multipathSigma = 0.0;    // m, extra white noise
    double multipathPeriod = 0.0;   // s; > 0: the offset switches on and off, half this period each
    Span ramp;                      // fixes drift away and back (no jump): the offset grows at rampRate inside
    double rampRate = 0.0;          // m/s, until rampMax, then decays at the same rate (direction drawn per seed)
    double rampMax = 0.0;           // m
    Span slip;                      // wheel speed reads (1 + slipRatio) times the truth
    double slipRatio = 0.0;
    double gyroBiasStepTime = -1.0; // s; a step change of the gyro bias at this time (< 0: none)
    double gyroBiasStep = 0.0;      // rad/s
    Span parked;                    // parked cars against the opposite kerb (unmapped objects for the scanner)
    double kerbClutter = 0.0;       // share of beams returning from spray or debris short of the true range
    Span kerbBlackout;              // no kerb scans (sensor fault or full occlusion)
    Span kerbShift;                 // a map error: the kerb on the scanner's left lies further out than mapped
    double kerbShiftDistance = 0.0; // m (a widened carriageway, a new lay-by)
};
// Named conditions. Development: nominal, gnss_outage, multipath, wheel_slip, imu_bias, delay_drop.
// Held out on R2 (different placements and magnitudes, plus a combination): the same names with suffix
// _h, and combined_h. Held out on R3 (new fault types): the names in conditionNamesR3(). Held out on R4
// for the second versions (new magnitudes and combinations, and a kerb map error): conditionNamesR4(). Held
// out on R5 for the stack's re-join after pose corrections: conditionNamesR5(). Held out on R6 for the
// verified re-join: conditionNamesR6().
SensorConditions sensorConditions(const std::string& name);
std::vector<std::string> conditionNames(bool heldOut);
std::vector<std::string> conditionNamesR3();
std::vector<std::string> conditionNamesR4();
std::vector<std::string> conditionNamesR5();
std::vector<std::string> conditionNamesR6();

struct Zone {
    double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
    bool contains(double x, double y) const { return x >= x0 && x <= x1 && y >= y0 && y <= y1; }
};
// An object standing in the carriageway that the kerb scanner can see (not in the map).
struct Obstacle {
    double cx = 0.0, cy = 0.0, yaw = 0.0, length = 4.5, width = 1.8;
};
struct SensorZones {
    std::vector<Zone> outage, multipath, slip, ramp, kerbBlackout, kerbShift;
    std::vector<Obstacle> obstacles;
};

// World side: truth in, measurements out. Deterministic for a seed. The kerb scanner sees the map's
// kerbs (shifted by the kerb deviation) and the obstacles; it uses its own random stream, so the other
// sensors' measurements do not depend on whether it exists.
class SensorSimulator {
public:
    SensorSimulator(const SensorConditions& cond, const SensorZones& zones, std::uint64_t seed, double wheelbase,
                    const LaneNetwork* map = nullptr);
    // Called every world tick (dt) with the true rear-axle state at time t. Generates the measurements
    // valid at t and returns, through `out`, every measurement whose arrival time is <= t.
    void step(double t, double dt, double x, double y, double yaw, double v, double steer, double accel, std::vector<Measurement>& out);
    bool gnssAvailable() const { return gnssAvailable_; }  // false inside an outage zone
    bool inMultipath() const { return inMultipath_; }
    bool inSlip() const { return inSlip_; }
    double gnssFault() const { return gnssFault_; }  // m, current multipath or ramp offset of the fixes (0: clean)
    const std::vector<Measurement>& log() const { return log_; }  // every measurement, in arrival order
    // One kerb scan from a true scanner pose, as the world produces it (also used by tests).
    std::vector<double> scan(double sx, double sy, double syaw);

private:
    struct Rng {
        std::uint64_t state;
        std::uint64_t next();
        double uniform(double a, double b);
        double normal();
    };
    double cadence(double rate) const;
    double kerbDeviationAt(double x, double y) const;
    SensorConditions c_;
    SensorZones zones_;
    Rng rng_;
    Rng kerbRng_;
    double wheelbase_;
    const LaneNetwork* map_;
    double gyroBias_, accelBias_, wheelScale_, steerOffset_;
    double corrX_ = 0.0, corrY_ = 0.0, mpX_ = 0.0, mpY_ = 0.0, rampX_ = 0.0, rampY_ = 0.0;
    double devPhase_[4] = {0.0, 0.0, 0.0, 0.0};
    double nextGnss_ = 0.0, nextWheel_ = 0.0, nextSteer_ = 0.0, nextImu_ = 0.0;
    double lastYawRate_ = 0.0;
    double mpSince_ = -1.0, rampInside_ = 0.0, rampBias_ = 0.0, gnssFault_ = 0.0;
    bool gnssAvailable_ = true, inMultipath_ = false, inSlip_ = false, stepped_ = false;
    std::vector<Measurement> pending_;
    std::vector<Measurement> log_;
};

std::string sensorLogCsv(const std::vector<Measurement>& log);
bool parseSensorLog(const std::string& text, std::vector<Measurement>& out, std::string& error);

// ---- Estimates and estimators ------------------------------------------------------------------------
struct StateEstimate {
    double t = 0.0;
    bool valid = false;
    double x = 0.0, y = 0.0, yaw = 0.0, v = 0.0;
    double accel = 0.0, yawRate = 0.0, steer = 0.0;
    bool hasCov = false;
    double pxx = 0.0, pxy = 0.0, pyy = 0.0, pxyaw = 0.0, pyyaw = 0.0, pyawyaw = 0.0, pvv = 0.0;
    const char* status = "INIT";
    const char* gnss = "-";      // GNSS integrity: TRUSTED, FAULTED (a jump odometry cannot explain), INCONSISTENT, OUTAGE
    const char* kerb = "-";      // kerb ranging: MATCHED (last scan used), REJECTED, NONE (no recent scan)
    double lastGnssUsed = -1.0;  // time of validity of the last GNSS fix the estimator used
    // Map-verified corrections: how many kerb registrations (re-localisations included) have moved the
    // estimate by more than 0.1 m, and the position change of the latest. A stack may follow such a
    // correction instead of steering at it; nothing else the estimator does is verified against the map.
    int verifiedCorrections = 0;
    double verifiedJumpX = 0.0, verifiedJumpY = 0.0;
    double sigmaPos() const;     // sqrt of the largest eigenvalue of the position covariance
    double sigmaAlong(double heading) const;  // position 1-sigma along / across a direction
    double sigmaAcross(double heading) const;
};

struct EstimatorStats {
    int gnssUsed = 0, gnssRejected = 0, wheelUsed = 0, wheelRejected = 0, resets = 0, lateDropped = 0, zeroUpdates = 0;
    int stepAlarms = 0, faultExits = 0, gnssExcluded = 0, reanchors = 0;  // GNSS integrity
    int faultRecoveries = 0, headingReanchors = 0;                        // second version
    int kerbUsed = 0, kerbRejected = 0, kerbRelocalisations = 0;          // kerb scans
    std::vector<double> nisGnss, nisWheel, nisKerb;  // normalised innovation squared of every candidate update
    std::vector<double> latencyUs;          // wall-clock processing time of each measurement (not deterministic)
};

// Tuned on the development split only, then frozen (LOCALISATION.md).
struct EstimatorConfig {
    double gnssSigma = 0.28;       // m, white measurement noise assumed per axis
    double gnssCorrSigma = 0.15;   // m, the receiver's slowly varying error, estimated as two states
    double gnssCorrTau = 60.0;     // s, its correlation time
    double wheelSigma = 0.03;      // m/s
    double gyroSigma = 0.0015;     // rad/s per sample
    double accelSigma = 0.03;      // m/s^2 per sample
    double posProcess = 0.0006;    // m^2 per metre travelled, unmodelled motion
    double gyroBiasWalk = 1e-4;    // rad/s per sqrt(s)
    double accelBiasWalk = 2e-4;   // m/s^2 per sqrt(s)
    double scaleWalk = 1e-5;       // per sqrt(s)
    double initPosSigma = 1.0;     // m
    double initYawSigma = 0.05;    // rad
    double initGyroBiasSigma = 0.01;   // rad/s
    double initAccelBiasSigma = 0.15;  // m/s^2
    double initScaleSigma = 0.02;
    double gateGnss = 11.0;        // chi-square, 2 dof, about 0.996
    double gateWheel = 10.83;      // chi-square, 1 dof, 0.999
    // Robust GNSS update: a fix whose NIS exceeds this (but passes the gate) has its noise inflated by
    // NIS / robustNis before the update, so near-gate outliers pull the estimate less. <= 0 disables.
    double robustNis = 3.0;        // chi-square, 2 dof, about 0.78
    int resetAfterRejected = 100;  // consecutive rejected fixes (10 s) before GNSS is trusted again ("ekf" only)
    double bufferSeconds = 1.0;    // delayed measurements older than this are dropped
    double cvAccelSigma = 1.5;     // m/s^2, GNSS-only constant-velocity filter
    // Antenna and IMU positions are vehicle installation data, known to every estimator.
    double antennaX = 1.60, antennaY = 0.0, imuLeverX = 1.40;

    // GNSS integrity ("ekf_int", "ekf_kerb"). No timer: rejected fixes are never trusted because time passed.
    bool integrity = false;
    double stepGate = 18.4;          // chi-square, 2 dof (99.99%): a fix-to-fix jump that odometry cannot explain
    double stepMaxGap = 2.0;         // s; fixes further apart are not compared
    int faultExitFixes = 5;          // consecutive fixes that fit the filter and not the detected fault, to clear it
    int inconsistentAfter = 20;      // consecutive rejected fixes with no detected jump: GNSS and filter disagree
    int reanchorWindow = 20;         // fixes that must agree with each other and with odometry before re-anchoring
    double reanchorScatter = 0.6;    // m RMS: the window's innovations about their mean (GNSS noise, not multipath)
    double reanchorInflation = 9.0;  // the filter's position variance may be this much too small (3 sigma)
    // Kerb ranging ("ekf_kerb"): scan-to-map registration against the mapped kerbs.
    bool kerb = false;
    double kerbSigma = 0.04;         // m per return: range noise and small-scale kerb roughness
    double kerbEffective = 15.0;     // returns per scan counted as independent
    // The real kerb deviates from the map smoothly, so every scan over the same stretch shares the same
    // pose error. Each scan's pose information carries this floor, counted once per kerbCorrLength of travel.
    double kerbFloorPos = 0.03;      // m
    double kerbFloorYaw = 0.004;     // rad
    double kerbCorrLength = 5.0;     // m
    // With kerbs the gyro bias is observable through heading, so the kerb filter lets it change faster
    // (development on R2: with 1e-4 a gyro-bias step left its heading 20-40 times overconfident).
    double kerbGyroBiasWalk = 3e-4;  // rad/s per sqrt(s), replaces gyroBiasWalk in "ekf_kerb"
    int kerbMinReturns = 8;          // matched returns needed to use a scan
    double kerbGate = 16.3;          // chi-square, 3 dof (99.9%), scan against the filter
    double kerbRobust = 7.8;         // chi-square, 3 dof (95%): above it the scan's weight is reduced

    // Second versions, after the R3 evaluation ("ekf_int2", "ekf_kerb2").
    // Recovery out of FAULTED: a jump back by the onset jump, or a gap in the fixes, ends the fault and
    // hands the decision to the re-anchoring test. There, the receiver's own track is evidence too: if over
    // the window it runs at an angle to the filter's track, the filter's heading has drifted, and the
    // offset bound for an overconfident filter no longer applies.
    bool recover = false;
    double headingEvidence = 0.07;   // rad between the two tracks (about 3 sigma of GNSS noise over 18 m)
    double headingEvidenceTrack = 10.0;  // m both tracks must cover
    // Kerb re-localisation: after relocAfter rejected scans, the scan is matched over a wider area around
    // the filter. A unique match with many returns is adopted, and the receiver must then earn trust again.
    bool relocalise = false;
    int relocAfter = 3;              // consecutive rejected scans
    double relocRadius = 6.0;        // m searched along and across the heading
    double relocStep = 0.5;          // m
};

class Estimator {
public:
    virtual ~Estimator() = default;
    virtual const char* name() const = 0;
    virtual void process(const Measurement& m) = 0;  // in arrival order
    virtual StateEstimate estimateAt(double t) const = 0;
    EstimatorStats stats;
};

// "ekf"      fused GNSS + IMU + wheel extended Kalman filter (9 states, delayed-measurement replay),
//            with the 10 s rejection reset; it ignores kerb scans (the estimator evaluated on R2)
// "ekf_int"  the same filter with GNSS integrity monitoring instead of the reset
// "ekf_kerb" ekf_int plus scan-to-map kerb registration
// "ekf_int2" ekf_int with a recovery path out of FAULTED and track-angle evidence for re-anchoring
// "ekf_kerb2" ekf_kerb with the same, plus kerb re-localisation over a wider search
// "dr"       wheel + gyro dead reckoning
// "gnss_hold" the latest GNSS fix, held
// "gnss_cv"  constant-velocity Kalman filter on GNSS fixes alone
std::unique_ptr<Estimator> makeEstimator(const std::string& name, const EstimatorConfig& cfg, const LaneNetwork& map);
std::vector<std::string> estimatorNames();          // the R1/R2 set: ekf, dr, gnss_cv, gnss_hold
std::vector<std::string> estimatorNamesExtended();  // ekf, ekf_int, ekf_kerb, dr, gnss_cv, gnss_hold
std::vector<std::string> estimatorNamesV2();        // ekf, ekf_int, ekf_int2, ekf_kerb, ekf_kerb2, dr, gnss_cv, gnss_hold
// "key=value,key=value" overrides of EstimatorConfig fields (used only for development tuning runs).
bool applyConfigOverrides(const std::string& text, EstimatorConfig& cfg, std::string& error);
std::string configText(const EstimatorConfig& cfg);

}  // namespace road

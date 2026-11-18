// World side of localisation: ground truth in, timestamped measurements out.
#include "road_driving/localisation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <stdexcept>

namespace road {

const char* kindName(Measurement::Kind k) {
    switch (k) {
        case Measurement::Gnss: return "GNSS";
        case Measurement::Imu: return "IMU";
        case Measurement::Wheel: return "WHEEL";
        case Measurement::Steer: return "STEER";
        case Measurement::Kerb: return "KERB";
    }
    return "?";
}

SensorConditions sensorConditions(const std::string& name) {
    SensorConditions c;
    c.name = name;
    const bool h = name.size() > 2 && name.compare(name.size() - 2, 2, "_h") == 0;
    const std::string base = h ? name.substr(0, name.size() - 2) : name;
    if (base == "nominal") {
    } else if (base == "gnss_outage") {
        c.outage = h ? Span{0.50, 0.80} : Span{0.35, 0.60};
    } else if (base == "multipath") {
        c.multipath = h ? Span{0.55, 0.75} : Span{0.30, 0.45};
        c.multipathBias = h ? 6.0 : 4.0;
        c.multipathSigma = h ? 1.5 : 1.0;
    } else if (base == "wheel_slip") {
        c.slip = h ? Span{0.40, 0.60} : Span{0.25, 0.40};
        c.slipRatio = h ? 0.30 : 0.20;
    } else if (base == "imu_bias") {
        c.imu.gyroBias = h ? 0.016 : 0.012;
        c.imu.accelBias = h ? 0.20 : 0.15;
        c.gyroBiasStepTime = h ? 45.0 : 30.0;
        c.gyroBiasStep = h ? 0.010 : 0.008;
    } else if (base == "delay_drop") {
        c.gnss.latency = h ? 0.50 : 0.40;
        c.gnss.latencyJitter = h ? 0.12 : 0.10;
        c.gnss.dropProb = h ? 0.40 : 0.30;
    } else if (name == "combined_h") {
        c.outage = {0.20, 0.40};
        c.multipath = {0.60, 0.75};
        c.multipathBias = 5.0;
        c.multipathSigma = 1.2;
        c.slip = {0.45, 0.55};
        c.slipRatio = 0.25;
        c.gnss.latency = 0.30;
        c.gnss.dropProb = 0.15;
    } else if (name == "test_persistent_bias") {
        // Tests only: a clean 6 m offset (no extra noise) for most of the route, far longer than 10 s.
        c.multipath = {0.25, 0.85};
        c.multipathBias = 6.0;
    } else if (name == "test_drift_recovery") {
        // Tests only: an unmodelled gyro-bias step inside an outage leaves the filter off and overconfident.
        c.outage = {0.30, 0.60};
        c.gyroBiasStepTime = 20.0;
        c.gyroBiasStep = 0.015;
    } else if (name == "nominal_r7") {
    } else if (name == "gnss_outage_r7") {
        c.outage = {0.25, 0.65};
    } else if (name == "multipath_r7") {
        c.multipath = {0.35, 0.60};
        c.multipathBias = 7.0;
        c.multipathSigma = 1.0;
    } else if (name == "kerb_return_r7") {
        c.kerbBlackout = {0.35, 0.65};
        c.ramp = {0.35, 0.65};
        c.rampRate = 0.35;
        c.rampMax = 5.0;
    } else if (name == "imu_fault_r7") {
        c.gyroBiasStepTime = 20.0;
        c.gyroBiasStep = 0.018;
        c.kerbBlackout = {0.20, 0.50};
        c.outage = {0.25, 0.50};
    } else if (name == "kerb_faults_r7") {
        c.kerbShift = {0.15, 0.35};
        c.kerbShiftDistance = 0.5;
        c.kerbBlackout = {0.45, 0.70};
        c.ramp = {0.45, 0.70};
        c.rampRate = 0.3;
        c.rampMax = 4.0;
    } else if (name == "combined_r7") {
        c.outage = {0.10, 0.25};
        c.multipath = {0.40, 0.55};
        c.multipathBias = 6.0;
        c.multipathSigma = 1.0;
        c.ramp = {0.60, 0.85};
        c.rampRate = 0.3;
        c.rampMax = 4.0;
        c.slip = {0.30, 0.40};
        c.slipRatio = 0.25;
        c.parked = {0.50, 0.70};
        c.kerbClutter = 0.10;
        c.gyroBiasStepTime = 45.0;
        c.gyroBiasStep = 0.012;
    } else if (name == "nominal_r6") {
    } else if (name == "gnss_outage_r6") {
        c.outage = {0.30, 0.70};
    } else if (name == "multipath_r6") {
        c.multipath = {0.25, 0.50};
        c.multipathBias = 8.0;
        c.multipathSigma = 1.2;
    } else if (name == "gnss_ramp_r6") {
        c.ramp = {0.20, 0.60};
        c.rampRate = 0.45;
        c.rampMax = 6.0;
    } else if (name == "kerb_return_r6") {
        c.kerbBlackout = {0.30, 0.60};
        c.ramp = {0.30, 0.60};
        c.rampRate = 0.3;
        c.rampMax = 4.0;
    } else if (name == "imu_fault_r6") {
        c.gyroBiasStepTime = 25.0;
        c.gyroBiasStep = 0.015;
        c.kerbBlackout = {0.25, 0.50};
        c.outage = {0.30, 0.50};
    } else if (name == "combined_r6") {
        c.outage = {0.10, 0.25};
        c.multipath = {0.40, 0.55};
        c.multipathBias = 7.0;
        c.multipathSigma = 1.0;
        c.ramp = {0.60, 0.85};
        c.rampRate = 0.3;
        c.rampMax = 4.0;
        c.slip = {0.30, 0.40};
        c.slipRatio = 0.25;
        c.kerbClutter = 0.10;
        c.gyroBiasStepTime = 50.0;
        c.gyroBiasStep = 0.010;
    } else if (name == "nominal_r5") {
    } else if (name == "gnss_outage_r5") {
        c.outage = {0.20, 0.55};
    } else if (name == "multipath_r5") {
        c.multipath = {0.30, 0.55};
        c.multipathBias = 9.0;
        c.multipathSigma = 1.5;
    } else if (name == "gnss_ramp_r5") {
        c.ramp = {0.30, 0.70};
        c.rampRate = 0.35;
        c.rampMax = 6.0;
    } else if (name == "imu_fault_r5") {
        c.gyroBiasStepTime = 30.0;
        c.gyroBiasStep = 0.012;
        c.outage = {0.35, 0.55};
    } else if (name == "kerb_faults_r5") {
        c.kerbShift = {0.10, 0.30};
        c.kerbShiftDistance = 0.4;
        c.kerbBlackout = {0.40, 0.70};
        c.multipath = {0.45, 0.60};
        c.multipathBias = 7.0;
        c.multipathSigma = 0.8;
    } else if (name == "combined_r5") {
        c.outage = {0.15, 0.30};
        c.ramp = {0.45, 0.75};
        c.rampRate = 0.3;
        c.rampMax = 4.0;
        c.slip = {0.35, 0.45};
        c.slipRatio = 0.25;
        c.kerbClutter = 0.10;
        c.parked = {0.50, 0.70};
        c.gyroBiasStepTime = 45.0;
        c.gyroBiasStep = 0.010;
    } else if (name == "nominal_r4") {
    } else if (name == "gnss_outage_r4") {
        c.outage = {0.25, 0.70};
    } else if (name == "multipath_r4") {
        c.multipath = {0.35, 0.60};
        c.multipathBias = 10.0;
        c.multipathSigma = 2.0;
    } else if (name == "gnss_ramp_r4") {
        c.ramp = {0.25, 0.65};
        c.rampRate = 0.4;
        c.rampMax = 7.0;
    } else if (name == "imu_fault_r4") {
        c.gyroBiasStepTime = 25.0;
        c.gyroBiasStep = 0.015;
        c.multipath = {0.30, 0.55};
        c.multipathBias = 6.0;
        c.multipathSigma = 1.0;
    } else if (name == "kerb_faults_r4") {
        c.kerbShift = {0.20, 0.45};
        c.kerbShiftDistance = 0.5;
        c.kerbBlackout = {0.55, 0.80};
        c.multipath = {0.60, 0.75};
        c.multipathBias = 8.0;
        c.multipathSigma = 0.5;
    } else if (name == "test_kerb_shift") {
        // Tests only: the kerb map error alone.
        c.kerbShift = {0.20, 0.50};
        c.kerbShiftDistance = 0.5;
    } else if (name == "combined_r4") {
        c.outage = {0.10, 0.25};
        c.ramp = {0.40, 0.70};
        c.rampRate = 0.3;
        c.rampMax = 5.0;
        c.slip = {0.50, 0.60};
        c.slipRatio = 0.30;
        c.kerbClutter = 0.10;
        c.gyroBiasStepTime = 40.0;
        c.gyroBiasStep = 0.008;
    } else if (name == "nominal_r3") {
    } else if (name == "gnss_outage_r3") {
        c.outage = {0.35, 0.75};
    } else if (name == "multipath_r3") {
        c.multipath = {0.40, 0.70};
        c.multipathBias = 8.0;
        c.multipathSigma = 1.0;
    } else if (name == "gnss_jumps_r3") {
        c.multipath = {0.30, 0.70};
        c.multipathBias = 3.0;
        c.multipathSigma = 0.5;
        c.multipathPeriod = 6.0;
    } else if (name == "gnss_ramp_r3") {
        c.ramp = {0.45, 0.80};
        c.rampRate = 0.25;
        c.rampMax = 5.0;
    } else if (name == "kerb_clutter_r3") {
        c.parked = {0.15, 0.65};
        c.kerbClutter = 0.15;
        c.kerbBlackout = {0.70, 0.80};
    } else if (name == "combined_r3") {
        c.outage = {0.15, 0.35};
        c.multipath = {0.55, 0.70};
        c.multipathBias = 7.0;
        c.multipathSigma = 1.0;
        c.slip = {0.40, 0.50};
        c.slipRatio = 0.25;
        c.parked = {0.10, 0.50};
        c.kerbClutter = 0.10;
        c.gyroBiasStepTime = 35.0;
        c.gyroBiasStep = 0.010;
    } else {
        throw std::invalid_argument("unknown sensor condition " + name);
    }
    return c;
}

std::vector<std::string> conditionNames(bool heldOut) {
    std::vector<std::string> base = {"nominal", "gnss_outage", "multipath", "wheel_slip", "imu_bias", "delay_drop"};
    if (heldOut) {
        for (std::string& s : base) {
            s += "_h";
        }
        base.push_back("combined_h");
    }
    return base;
}

std::vector<std::string> conditionNamesR3() {
    return {"nominal_r3", "gnss_outage_r3", "multipath_r3", "gnss_jumps_r3", "gnss_ramp_r3", "kerb_clutter_r3", "combined_r3"};
}

std::vector<std::string> conditionNamesR7() {
    return {"nominal_r7", "gnss_outage_r7", "multipath_r7", "kerb_return_r7", "imu_fault_r7", "kerb_faults_r7", "combined_r7"};
}

std::vector<std::string> conditionNamesR6() {
    return {"nominal_r6", "gnss_outage_r6", "multipath_r6", "gnss_ramp_r6", "kerb_return_r6", "imu_fault_r6", "combined_r6"};
}

std::vector<std::string> conditionNamesR5() {
    return {"nominal_r5", "gnss_outage_r5", "multipath_r5", "gnss_ramp_r5", "imu_fault_r5", "kerb_faults_r5", "combined_r5"};
}

std::vector<std::string> conditionNamesR4() {
    return {"nominal_r4", "gnss_outage_r4", "multipath_r4", "gnss_ramp_r4", "imu_fault_r4", "kerb_faults_r4", "combined_r4"};
}

std::uint64_t SensorSimulator::Rng::next() {
    std::uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

double SensorSimulator::Rng::uniform(double a, double b) {
    return a + (b - a) * (static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0));
}

double SensorSimulator::Rng::normal() {
    // Box-Muller from two uniforms in (0, 1].
    const double u1 = 1.0 - uniform(0.0, 1.0);
    const double u2 = uniform(0.0, 1.0);
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}

SensorSimulator::SensorSimulator(const SensorConditions& cond, const SensorZones& zones, std::uint64_t seed, double wheelbase,
                                 const LaneNetwork* map)
    : c_(cond), zones_(zones), rng_{seed ^ 0x5E5B0A11D1E5ULL}, kerbRng_{seed ^ 0x4B45524253434E31ULL}, wheelbase_(wheelbase), map_(map) {
    auto sign = [&]() { return rng_.uniform(0.0, 1.0) < 0.5 ? -1.0 : 1.0; };
    gyroBias_ = sign() * c_.imu.gyroBias;
    accelBias_ = sign() * c_.imu.accelBias;
    wheelScale_ = sign() * c_.wheel.scaleError;
    steerOffset_ = sign() * c_.steer.offset;
    const double mpAngle = rng_.uniform(0.0, 2.0 * kPi);
    mpX_ = c_.multipathBias * std::cos(mpAngle);
    mpY_ = c_.multipathBias * std::sin(mpAngle);
    corrX_ = c_.gnss.corrSigma * rng_.normal();
    corrY_ = c_.gnss.corrSigma * rng_.normal();
    // Everything added later draws from its own stream, so the sensors above are unchanged by it.
    for (double& p : devPhase_) {
        p = kerbRng_.uniform(0.0, 2.0 * kPi);
    }
    const double rampAngle = kerbRng_.uniform(0.0, 2.0 * kPi);
    rampX_ = std::cos(rampAngle);
    rampY_ = std::sin(rampAngle);
}

double SensorSimulator::kerbDeviationAt(double x, double y) const {
    const double l = c_.kerb.deviationWavelength;
    const double u1 = x * std::cos(devPhase_[0]) + y * std::sin(devPhase_[0]);
    const double u2 = x * std::cos(devPhase_[1]) + y * std::sin(devPhase_[1]);
    return c_.kerb.kerbDeviation * (std::sin(2.0 * kPi * u1 / l + devPhase_[2]) + std::sin(2.0 * kPi * u2 / (0.61 * l) + devPhase_[3]));
}

std::vector<double> SensorSimulator::scan(double sx, double sy, double syaw) {
    const KerbConfig& k = c_.kerb;
    std::vector<double> out(static_cast<size_t>(k.beams), -1.0);
    for (int i = 0; i < k.beams; ++i) {
        // A fixed number of draws per beam keeps the stream aligned across conditions.
        const double uClutter = kerbRng_.uniform(0.0, 1.0), uWhere = kerbRng_.uniform(0.0, 1.0), uDrop = kerbRng_.uniform(0.0, 1.0);
        const double noise = kerbRng_.normal();
        const double th = syaw + k.firstBearing + i * k.bearingStep, dx = std::cos(th), dy = std::sin(th);
        double r = map_ ? map_->rayToKerb(sx, sy, dx, dy, k.maxRange + 1.0) : -1.0;
        if (r > 0.0) {
            // The real kerb lies off the mapped one by the deviation, along its normal.
            const double hx = sx + r * dx, hy = sy + r * dy;
            double nx = 0.0, ny = 0.0;
            map_->kerbDistance(hx, hy, nx, ny);
            const double incidence = std::max(0.15, -(dx * nx + dy * ny));
            r += kerbDeviationAt(hx, hy) / incidence;
            // A kerb the map has wrong: inside a shift zone the kerb on the scanner's left lies further out.
            if (c_.kerbShiftDistance != 0.0 && std::cos(syaw) * (hy - sy) - std::sin(syaw) * (hx - sx) > 0.0) {
                for (const Zone& z : zones_.kerbShift) {
                    if (z.contains(hx, hy)) {
                        r += c_.kerbShiftDistance / incidence;
                        break;
                    }
                }
            }
        }
        for (const Obstacle& o : zones_.obstacles) {
            // Slab test in the obstacle's frame.
            const double c = std::cos(o.yaw), s = std::sin(o.yaw);
            const double px = c * (sx - o.cx) + s * (sy - o.cy), py = -s * (sx - o.cx) + c * (sy - o.cy);
            const double qx = c * dx + s * dy, qy = -s * dx + c * dy;
            double t0 = -1e18, t1 = 1e18;
            bool miss = false;
            for (int a = 0; a < 2 && !miss; ++a) {
                const double p = a == 0 ? px : py, q = a == 0 ? qx : qy, h = 0.5 * (a == 0 ? o.length : o.width);
                if (std::fabs(q) < 1e-12) {
                    miss = std::fabs(p) > h;
                } else {
                    const double ta = (-h - p) / q, tb = (h - p) / q;
                    t0 = std::max(t0, std::min(ta, tb));
                    t1 = std::min(t1, std::max(ta, tb));
                }
            }
            if (!miss && t1 >= t0 && t0 > 0.0 && (r < 0.0 || t0 < r)) {
                r = t0;
            }
        }
        if (uClutter < c_.kerbClutter) {
            r = k.minRange + uWhere * ((r > 0.0 ? r : k.maxRange) - k.minRange);  // spray or debris short of the target
        }
        if (r > 0.0) {
            r += k.rangeNoise * noise;
        }
        if (uDrop < k.dropout || r < k.minRange || r > k.maxRange) {
            r = -1.0;
        }
        out[static_cast<size_t>(i)] = r;
    }
    return out;
}

void SensorSimulator::step(double t, double dt, double x, double y, double yaw, double v, double steer, double accel,
                           std::vector<Measurement>& out) {
    const long k = std::lround(t / dt);
    const long imuEvery = std::max(1L, std::lround(1.0 / (c_.imu.rate * dt)));
    const long wheelEvery = std::max(1L, std::lround(1.0 / (c_.wheel.rate * dt)));
    const long steerEvery = std::max(1L, std::lround(1.0 / (c_.steer.rate * dt)));
    const long gnssEvery = std::max(1L, std::lround(1.0 / (c_.gnss.rate * dt)));
    auto inside = [&](const std::vector<Zone>& zs) {
        for (const Zone& z : zs) {
            if (z.contains(x, y)) {
                return true;
            }
        }
        return false;
    };
    gnssAvailable_ = !inside(zones_.outage);
    inMultipath_ = inside(zones_.multipath);
    inSlip_ = inside(zones_.slip);
    if (inMultipath_ && mpSince_ < 0.0) {
        mpSince_ = t;
    } else if (!inMultipath_) {
        mpSince_ = -1.0;
    }
    // Intermittent multipath: the offset switches on and off, half a period each, from zone entry.
    const bool mpOn = inMultipath_ && (c_.multipathPeriod <= 0.0 ||
                                       static_cast<long>(std::floor((t - mpSince_) / (0.5 * c_.multipathPeriod))) % 2 == 0);
    // Slow excursion: inside the zone the offset grows at rampRate until rampMax, then decays at the same rate,
    // also after the zone; it never steps.
    if (c_.rampRate > 0.0) {
        const bool rising = inside(zones_.ramp) && rampInside_ * c_.rampRate < c_.rampMax;
        if (inside(zones_.ramp)) {
            rampInside_ += dt;
        }
        rampBias_ = std::max(0.0, std::min(c_.rampMax, rampBias_ + (rising ? 1.0 : -1.0) * c_.rampRate * dt));
    }
    const double rampBias = rampBias_;
    gnssFault_ = (mpOn ? c_.multipathBias : 0.0) + rampBias;

    // Slowly varying errors: bias random walks, a scheduled gyro-bias step, correlated GNSS error.
    if (stepped_) {
        gyroBias_ += c_.imu.gyroBiasWalk * std::sqrt(dt) * rng_.normal();
        accelBias_ += c_.imu.accelBiasWalk * std::sqrt(dt) * rng_.normal();
        const double phi = std::exp(-dt / c_.gnss.corrTau);
        const double q = c_.gnss.corrSigma * std::sqrt(1.0 - phi * phi);
        corrX_ = phi * corrX_ + q * rng_.normal();
        corrY_ = phi * corrY_ + q * rng_.normal();
        if (c_.gyroBiasStepTime >= 0.0 && t - dt < c_.gyroBiasStepTime && t >= c_.gyroBiasStepTime) {
            gyroBias_ += c_.gyroBiasStep;
        }
    }
    // At rest under a brake-hold command the model's acceleration state is the command, not motion: the
    // accelerometer measures the physical acceleration, which is zero while the car stands still.
    accel = (v <= 1e-9 && accel < 0.0) ? 0.0 : accel;
    const double yawRate = v * std::tan(steer) / wheelbase_;
    const double yawAccel = stepped_ ? (yawRate - lastYawRate_) / dt : 0.0;
    lastYawRate_ = yawRate;
    stepped_ = true;

    if (k % imuEvery == 0) {
        const double lx = c_.imu.leverX;
        Measurement m;
        m.kind = Measurement::Imu;
        m.tMeas = m.tArrival = t;
        m.a = accel - yawRate * yawRate * lx + accelBias_ + c_.imu.accelNoise * rng_.normal();
        m.b = v * yawRate + yawAccel * lx + c_.imu.accelNoise * rng_.normal();
        m.c = yawRate + gyroBias_ + c_.imu.gyroNoise * rng_.normal();
        pending_.push_back(m);
    }
    if (k % wheelEvery == 0) {
        Measurement m;
        m.kind = Measurement::Wheel;
        m.tMeas = t;
        m.tArrival = t + c_.wheel.latency;
        const double noise = rng_.normal();
        double speed = 0.0;  // the encoders produce no ticks at rest
        if (v > 0.01) {
            speed = v * (1.0 + wheelScale_) * (inSlip_ ? 1.0 + c_.slipRatio : 1.0) + c_.wheel.noise * noise;
            speed = std::max(0.0, std::round(speed / c_.wheel.quantum) * c_.wheel.quantum);
        }
        m.a = speed;
        pending_.push_back(m);
    }
    if (k % steerEvery == 0) {
        Measurement m;
        m.kind = Measurement::Steer;
        m.tMeas = t;
        m.tArrival = t + c_.steer.latency;
        m.a = steer + steerOffset_ + c_.steer.noise * rng_.normal();
        pending_.push_back(m);
    }
    if (k % gnssEvery == 0) {
        const double drop = rng_.uniform(0.0, 1.0);
        const double jitter = rng_.uniform(-c_.gnss.latencyJitter, c_.gnss.latencyJitter);
        const double nx = rng_.normal(), ny = rng_.normal(), mx = rng_.normal(), my = rng_.normal();
        if (gnssAvailable_ && drop >= c_.gnss.dropProb) {
            const double cy = std::cos(yaw), sy = std::sin(yaw);
            Measurement m;
            m.kind = Measurement::Gnss;
            m.tMeas = t;
            m.tArrival = t + std::max(0.0, c_.gnss.latency + jitter);
            m.a = x + cy * c_.gnss.antennaX - sy * c_.gnss.antennaY + c_.gnss.sigma * nx + corrX_;
            m.b = y + sy * c_.gnss.antennaX + cy * c_.gnss.antennaY + c_.gnss.sigma * ny + corrY_;
            if (mpOn) {
                m.a += mpX_ + c_.multipathSigma * mx;
                m.b += mpY_ + c_.multipathSigma * my;
            }
            m.a += rampBias * rampX_;
            m.b += rampBias * rampY_;
            m.c = c_.gnss.reportedSigma;
            pending_.push_back(m);
        }
    }
    const long kerbEvery = std::max(1L, std::lround(1.0 / (c_.kerb.rate * dt)));
    if (map_ && k % kerbEvery == 0 && !inside(zones_.kerbBlackout)) {
        Measurement m;
        m.kind = Measurement::Kerb;
        m.tMeas = t;
        m.tArrival = t + c_.kerb.latency;
        m.a = c_.kerb.firstBearing;
        m.b = c_.kerb.bearingStep;
        m.c = c_.kerb.mountX;
        m.ranges = scan(x + c_.kerb.mountX * std::cos(yaw), y + c_.kerb.mountX * std::sin(yaw), yaw);
        pending_.push_back(m);
    }

    // Round every new measurement through the log's text format, so that an estimator fed from the
    // world and one replayed from sensors.csv see bit-identical inputs.
    for (Measurement& m : pending_) {
        char buf[192];
        std::snprintf(buf, sizeof buf, "%.4f %.4f %.9g %.9g %.9g", m.tMeas, m.tArrival, m.a, m.b, m.c);
        std::istringstream rs(buf);
        rs >> m.tMeas >> m.tArrival >> m.a >> m.b >> m.c;
        for (double& r : m.ranges) {
            std::snprintf(buf, sizeof buf, "%.4f", r);
            r = std::strtod(buf, nullptr);
        }
    }

    // Release everything that has arrived, in a fixed order.
    std::vector<Measurement> ready;
    std::vector<Measurement> keep;
    for (const Measurement& m : pending_) {
        (m.tArrival <= t + 1e-9 ? ready : keep).push_back(m);
    }
    pending_.swap(keep);
    std::stable_sort(ready.begin(), ready.end(), [](const Measurement& a, const Measurement& b) {
        if (a.tArrival != b.tArrival) return a.tArrival < b.tArrival;
        if (a.kind != b.kind) return a.kind < b.kind;
        return a.tMeas < b.tMeas;
    });
    for (const Measurement& m : ready) {
        out.push_back(m);
        log_.push_back(m);
    }
}

std::string sensorLogCsv(const std::vector<Measurement>& log) {
    std::string s = "t_meas,t_arrival,sensor,a,b,c,ranges\n";
    char line[192];
    for (const Measurement& m : log) {
        std::snprintf(line, sizeof line, "%.4f,%.4f,%s,%.9g,%.9g,%.9g", m.tMeas, m.tArrival, kindName(m.kind), m.a, m.b, m.c);
        s += line;
        if (m.kind == Measurement::Kerb) {
            s += ',';
            for (size_t i = 0; i < m.ranges.size(); ++i) {
                std::snprintf(line, sizeof line, i ? " %.4f" : "%.4f", m.ranges[i]);
                s += line;
            }
        }
        s += '\n';
    }
    return s;
}

bool parseSensorLog(const std::string& text, std::vector<Measurement>& out, std::string& error) {
    std::istringstream in(text);
    std::string line;
    std::getline(in, line);  // header
    int n = 1;
    while (std::getline(in, line)) {
        ++n;
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty()) {
            continue;
        }
        std::istringstream ls(line);
        std::string f[7];
        for (std::string& s : f) {
            std::getline(ls, s, ',');
        }
        Measurement m;
        if (f[2] == "GNSS") m.kind = Measurement::Gnss;
        else if (f[2] == "IMU") m.kind = Measurement::Imu;
        else if (f[2] == "WHEEL") m.kind = Measurement::Wheel;
        else if (f[2] == "STEER") m.kind = Measurement::Steer;
        else if (f[2] == "KERB") m.kind = Measurement::Kerb;
        else {
            error = "line " + std::to_string(n) + ": unknown sensor " + f[2];
            return false;
        }
        try {
            m.tMeas = std::stod(f[0]);
            m.tArrival = std::stod(f[1]);
            m.a = std::stod(f[3]);
            m.b = std::stod(f[4]);
            m.c = std::stod(f[5]);
        } catch (const std::exception&) {
            error = "line " + std::to_string(n) + ": bad number";
            return false;
        }
        if (m.kind == Measurement::Kerb) {
            const char* p = f[6].c_str();
            while (*p) {
                char* end = nullptr;
                const double r = std::strtod(p, &end);
                if (end == p) break;
                m.ranges.push_back(r);
                p = end;
            }
        }
        out.push_back(m);
    }
    return true;
}

}  // namespace road

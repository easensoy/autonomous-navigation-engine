// World side of localisation: ground truth in, timestamped measurements out.
#include "road_driving/localisation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <stdexcept>

namespace road {

const char* kindName(Measurement::Kind k) {
    switch (k) {
        case Measurement::Gnss: return "GNSS";
        case Measurement::Imu: return "IMU";
        case Measurement::Wheel: return "WHEEL";
        case Measurement::Steer: return "STEER";
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

SensorSimulator::SensorSimulator(const SensorConditions& cond, const SensorZones& zones, std::uint64_t seed, double wheelbase)
    : c_(cond), zones_(zones), rng_{seed ^ 0x5E5B0A11D1E5ULL}, wheelbase_(wheelbase) {
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
            if (inMultipath_) {
                m.a += mpX_ + c_.multipathSigma * mx;
                m.b += mpY_ + c_.multipathSigma * my;
            }
            m.c = c_.gnss.reportedSigma;
            pending_.push_back(m);
        }
    }

    // Round every new measurement through the log's text format, so that an estimator fed from the
    // world and one replayed from sensors.csv see bit-identical inputs.
    for (Measurement& m : pending_) {
        char buf[192];
        std::snprintf(buf, sizeof buf, "%.4f %.4f %.9g %.9g %.9g", m.tMeas, m.tArrival, m.a, m.b, m.c);
        std::istringstream rs(buf);
        rs >> m.tMeas >> m.tArrival >> m.a >> m.b >> m.c;
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
    std::string s = "t_meas,t_arrival,sensor,a,b,c\n";
    char line[192];
    for (const Measurement& m : log) {
        std::snprintf(line, sizeof line, "%.4f,%.4f,%s,%.9g,%.9g,%.9g\n", m.tMeas, m.tArrival, kindName(m.kind), m.a, m.b, m.c);
        s += line;
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
        std::string f[6];
        for (std::string& s : f) {
            std::getline(ls, s, ',');
        }
        Measurement m;
        if (f[2] == "GNSS") m.kind = Measurement::Gnss;
        else if (f[2] == "IMU") m.kind = Measurement::Imu;
        else if (f[2] == "WHEEL") m.kind = Measurement::Wheel;
        else if (f[2] == "STEER") m.kind = Measurement::Steer;
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
        out.push_back(m);
    }
    return true;
}

}  // namespace road

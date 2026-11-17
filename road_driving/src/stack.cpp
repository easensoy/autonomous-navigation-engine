#include "stack.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace road {

using namespace stackparams;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kEnvDecel = 1.6;       // m/s^2: planning deceleration ahead of speed caps
constexpr double kHorizon = 150.0;      // m: speed planning horizon
constexpr double kPreview = 0.40;       // s: speed preview, compensates the command delay and lag
constexpr double kKp = 1.2;             // 1/s: speed feedback gain
constexpr double kLookBase = 2.0;       // m: pure-pursuit look-ahead at rest
constexpr double kLookGain = 0.30;      // s: look-ahead growth with speed
constexpr double kLookMax = 7.5;        // m
constexpr double kSteerPredict = 0.10;  // s: steering uses the pose predicted this far ahead
constexpr double kHoldDecel = 2.0;      // m/s^2: brake hold at rest

std::string fmt(double v, int prec = 2) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
}

// The constant deceleration command A that stops the car within d metres, given its speed v and actual
// acceleration a. The model is the vehicle's own actuator model: the current acceleration holds for the
// command delay Td, then relaxes towards -A with the longitudinal lag tau. Its stopping distance is
//   d = v Td + a Td^2 / 2 + w^2 / (2A) - (a + A) tau^2,   with   w = v + a Td + (a + A) tau,
// exact once the stop lasts several tau. Solved for A this is the positive root
//   A = (-E + sqrt(E^2 + tau^2 c^2)) / tau^2,   c = v + a (Td + tau),   E = d - v Td - a Td^2 / 2 + a tau^2 - c tau.
double stopDecel(double d, double v, double a, double delay, double lag) {
    const double c = v + a * (delay + lag);
    if (c <= 0.05 || v <= 0.01) {
        return 0.0;  // at rest, or already slowing to rest within the latency
    }
    const double e = d - v * delay - 0.5 * a * delay * delay + a * lag * lag - c * lag;
    return (-e + std::sqrt(e * e + lag * lag * c * c)) / (lag * lag);
}

}  // namespace

std::vector<double> speedCaps(const DrivePath& path) {
    std::vector<double> caps;
    caps.reserve(path.samples().size());
    for (const PathSample& p : path.samples()) {
        double cap = p.speedLimit;
        if (std::fabs(p.kappa) > 1e-9) {
            cap = std::min(cap, std::sqrt(kLatPlan / std::fabs(p.kappa)));
        }
        caps.push_back(cap);
    }
    return caps;
}

double requiredDecel(const DrivePath& path, const std::vector<double>& caps, double s, double v) {
    double a = 0.0;
    const std::vector<PathSample>& smp = path.samples();
    for (size_t i = path.sampleIndexAt(s); i < smp.size() && smp[i].s <= s + kHorizon; ++i) {
        const double cap = caps[i] + kCapTolerance;
        if (v <= cap) {
            continue;
        }
        const double d = smp[i].s - s - v * kLatency;
        if (d <= 0.05) {
            if (v > cap + 0.3) {
                return kInf;  // the cap lies inside the latency distance: nothing can meet it now
            }
            continue;
        }
        a = std::max(a, (v * v - cap * cap) / (2.0 * d));
    }
    return a;
}

Stack::Stack(const LaneNetwork& net, const VehicleParams& vp, Variant variant, int goalElement, double goalLaneS)
    : net_(net), vp_(vp), variant_(variant), goal_(goalElement), goalLaneS_(goalLaneS), router_(net) {}

const char* Stack::modeName() const {
    switch (mode_) {
        case Mode::Drive: return "DRIVE";
        case Mode::StopForClosure: return "STOP_FOR_CLOSURE";
        case Mode::Fallback: return "FALLBACK";
        case Mode::Hold: return "HOLD_AT_CLOSURE";
        case Mode::HoldFallback: return "HOLD_AFTER_FALLBACK";
        case Mode::Done: return "DONE";
        case Mode::NoRoute: return "NO_ROUTE";
        case Mode::LocalisationStop: return "LOCALISATION_STOP";
    }
    return "?";
}

void Stack::event(const std::string& type, const std::string& detail) { events.push_back({t_, "STACK", type, detail}); }

int Stack::currentElement() const { return path_.elements().at(idx_); }

void Stack::track(const VehicleState& pose) {
    v_ = pose.v;
    a_ = pose.accel;
    if (path_.empty()) {
        return;
    }
    const DrivePath::Projection pr = path_.project(pose.x, pose.y, s_, 2.0, 4.0 + pose.v);
    if (pr.ok) {
        s_ = pr.s;
        lateral_ = pr.lateral;
    }
    // Step rule: the engine advances one node each time the rear axle enters the next route element.
    const size_t idx = path_.elementIndexAt(s_);
    while (idx_ < idx) {
        ++idx_;
        const int e = path_.elements()[idx_];
        if (router_.active() && !engineStale_) {
            router_.step();
            if (router_.position() != e) {
                engineStale_ = true;
                event("ENGINE_DESYNC", "engine=" + net_.elements[static_cast<size_t>(router_.position())].name +
                                           ";vehicle=" + net_.elements[static_cast<size_t>(e)].name);
            }
        } else {
            engineStale_ = true;  // BLOCKED: the engine stays where it stopped; recovery re-plans from the actual element
        }
    }
}

void Stack::adopt(const DrivePath& path, double s, const std::string& reason) {
    path_ = path;
    caps_ = speedCaps(path_);
    s_ = s;
    idx_ = 0;
    const int gi = path_.indexOf(goal_);
    goalOnPath_ = gi >= 0;
    if (goalOnPath_) {
        goalS_ = path_.elementStart(static_cast<size_t>(gi)) + (goalLaneS_ - net_.elements[static_cast<size_t>(goal_)].laneS0);
    }
    goalLatched_ = false;
    routes.push_back({t_, reason, router_.state(), path_.elements()});
    computeEnvelope();
}

double Stack::barrierS(const DrivePath& path, const ClosureReport& c) const {
    const int seg = net_.segmentAt(c.lane, c.barrierLaneS);
    const int i = path.indexOf(seg);
    if (i < 0) {
        return -1.0;
    }
    return path.elementStart(static_cast<size_t>(i)) + (c.barrierLaneS - net_.elements[static_cast<size_t>(seg)].laneS0);
}

double Stack::nearestBarrierAhead(const DrivePath& path, double sRear) const {
    double best = -1.0;
    const double front = sRear + vp_.frontOffset();
    for (const ClosureReport& c : known_) {
        const double sb = barrierS(path, c);
        if (sb < 0.0 || front >= sb + c.worksLength) {
            continue;
        }
        if (best < 0.0 || sb < best) {
            best = sb;
        }
    }
    return best;
}

bool Stack::entersKnownClosure(const DrivePath& path, double sRear) const { return nearestBarrierAhead(path, sRear) >= 0.0; }

void Stack::initialise(const Observation& obs) {
    t_ = obs.t;
    v_ = obs.pose.v;
    const int start = associate(net_, obs.pose.x, obs.pose.y, obs.pose.yaw);
    for (const ClosureReport& c : obs.closures) {
        if (!c.active) {
            continue;
        }
        known_.push_back(c);
        router_.reportClosed(c.closed);
        event("CLOSURE_OBSERVED", "id=" + c.id + ";kind=" + c.kind + ";source=ORACLE;when=before-departure");
    }
    if (start < 0 || !router_.start(start, goal_)) {
        mode_ = Mode::NoRoute;
        if (start >= 0) {
            path_ = DrivePath(net_, {start});
            caps_ = speedCaps(path_);
        }
        event("NO_ROUTE", "engine=" + router_.state());
        return;
    }
    const DrivePath p(net_, router_.routeAhead());
    const DrivePath::Projection pr = p.project(obs.pose.x, obs.pose.y, 0.0, 0.0, net_.elements[static_cast<size_t>(start)].length);
    adopt(p, pr.ok ? pr.s : 0.0, "initial");
    event("ROUTE_PLANNED", "elements=" + std::to_string(p.elements().size()) + ";engine=" + router_.state());
    if (entersKnownClosure(path_, s_)) {
        planClosureStop("closure on the start element", v_);  // the engine exempts its start node (N2)
    }
}

void Stack::plan(const Observation& obs) {
    t_ = obs.t;
    track(obs.pose);
    if (mode_ == Mode::Done || mode_ == Mode::HoldFallback || mode_ == Mode::NoRoute) {
        return;
    }

    for (const ClosureReport& c : obs.closures) {
        auto it = std::find_if(known_.begin(), known_.end(), [&](const ClosureReport& k) { return k.id == c.id; });
        if (c.active && it == known_.end()) {
            known_.push_back(c);
            const int cur = currentElement();
            const double sb = barrierS(path_, c);
            std::string d = "id=" + c.id + ";kind=" + c.kind + ";source=ORACLE;v=" + fmt(v_) + ";element=" +
                            net_.elements[static_cast<size_t>(cur)].name;
            if (sb >= 0.0) {
                d += ";front_to_barrier=" + fmt(sb - s_ - vp_.frontOffset());
            }
            event("CLOSURE_OBSERVED", d);
            router_.reportClosed(c.closed);  // NavigationCore replans from its current node, or BLOCKED
            event("ENGINE_REPORT", "state=" + router_.state());
            if (mode_ == Mode::Fallback) {
                continue;
            }
            bool same = router_.active() && !engineStale_;
            if (same) {
                const std::vector<int> ahead = router_.routeAhead();
                same = ahead == std::vector<int>(path_.elements().begin() + static_cast<std::ptrdiff_t>(idx_), path_.elements().end());
            }
            if (entersKnownClosure(path_, s_) || !same) {
                replan("closure " + c.id, v_);
            } else {
                event("ROUTE_KEPT", "closure not on the route ahead");
            }
        } else if (!c.active && it != known_.end()) {
            known_.erase(it);
            router_.clear(c.closed);
            event("CLOSURE_LIFT_OBSERVED", "id=" + c.id + ";source=ORACLE");
            if (mode_ == Mode::Hold || mode_ == Mode::StopForClosure) {
                recover("lift " + c.id);
            }
        }
    }

    // A commitment mask only matters on its junction's approach lane; clear it once the car has left that lane.
    for (size_t i = 0; i < masks_.size();) {
        if (net_.elements[static_cast<size_t>(currentElement())].lane != masks_[i].approachLane) {
            router_.clear({masks_[i].connector});
            event("MASK_CLEARED", "connector=" + net_.elements[static_cast<size_t>(masks_[i].connector)].name);
            masks_.erase(masks_.begin() + static_cast<std::ptrdiff_t>(i));
            if (mode_ == Mode::StopForClosure || mode_ == Mode::Hold) {
                recover("mask cleared");
            }
        } else {
            ++i;
        }
    }

    if (mode_ == Mode::StopForClosure && v_ < 0.02 && stopLatched_) {
        mode_ = Mode::Hold;
        const double sb = nearestBarrierAhead(path_, s_);
        event("STOPPED", "front_to_barrier=" + fmt(sb >= 0.0 ? sb - s_ - vp_.frontOffset() : -1.0, 3));
    }
    if (mode_ == Mode::Fallback && v_ < 0.02) {
        mode_ = Mode::HoldFallback;
        event("STOPPED", "after fallback");
    }
    if (mode_ == Mode::Drive && goalOnPath_ && std::fabs(goalS_ - s_) < 1.0 && v_ < 0.05) {
        mode_ = Mode::Done;
        event("GOAL_REACHED", "error=" + fmt(s_ - goalS_, 3));
    }

    // Constraint monitor over the stopping horizon.
    if (mode_ == Mode::Drive || mode_ == Mode::StopForClosure) {
        const double aReq = requiredDecel(path_, caps_, s_, v_);
        if (mode_ == Mode::Drive && aReq > vp_.decelFirm) {
            fallback("speed caps ahead need " + (std::isfinite(aReq) ? fmt(aReq) : std::string("inf")) +
                     " m/s^2, above the firm limit");
        } else if (std::fabs(lateral_) > 1.0) {
            fallback("lateral error " + fmt(lateral_) + " m");
        } else if (mode_ == Mode::Drive && entersKnownClosure(path_, s_)) {
            planClosureStop("monitor: known closure ahead", v_);
        }
        aEnv_ = std::min(vp_.decelFirm, std::max(kEnvDecel, 1.05 * (std::isfinite(aReq) ? aReq : vp_.decelFirm)));
    }
    localisationPolicy(obs);
    computeEnvelope();
}

void Stack::localisationPolicy(const Observation& obs) {
    using namespace locpolicy;
    if (!obs.estimated) {
        return;
    }
    const std::string detail = "sigma_across=" + fmt(obs.sigmaAcross, 3) + ";sigma_along=" + fmt(obs.sigmaAlong, 3) +
                               ";valid=" + (obs.estimateValid ? "1" : "0") + ";age=" + fmt(obs.estimateAge, 3);
    const bool unreliable = !obs.estimateValid || obs.estimateAge > kMaxAge || obs.sigmaAcross > kStopAcross || obs.sigmaAlong > kStopAlong;
    const bool degraded = obs.sigmaAcross > kSlowAcross || obs.sigmaAlong > kSlowAlong;
    if (degraded != degraded_) {
        degraded_ = degraded;
        event(degraded ? "LOCALISATION_DEGRADED" : "LOCALISATION_NOMINAL", detail);
    }
    speedCap_ = degraded ? kDegradedSpeed : 1e9;
    if (mode_ == Mode::LocalisationStop) {
        if (unreliable || obs.sigmaAcross >= kResumeAcross) {
            goodSince_ = -1.0;
        } else if (goodSince_ < 0.0) {
            goodSince_ = t_;
        } else if (t_ - goodSince_ >= 2.0) {
            mode_ = beforeLocStop_;  // a planned closure stop, if any, is still in force
            goodSince_ = -1.0;
            event("LOCALISATION_RECOVERED", detail);
        }
        return;
    }
    if (unreliable && (mode_ == Mode::Drive || mode_ == Mode::StopForClosure)) {
        beforeLocStop_ = mode_;
        mode_ = Mode::LocalisationStop;
        goodSince_ = -1.0;
        event("LOCALISATION_UNRELIABLE", detail);
    }
}

void Stack::replan(const std::string& reason, double v) {
    const int cur = currentElement();
    const std::vector<int> old(path_.elements().begin() + static_cast<std::ptrdiff_t>(idx_), path_.elements().end());
    const double offset = s_ - path_.elementStart(idx_);  // the car's position within its element
    for (int iter = 0; iter < 6; ++iter) {
        std::vector<int> cand;
        if (router_.active() && !engineStale_ && router_.consistent()) {
            cand = router_.routeAhead();
        }
        if (cand.empty() || cand.front() != cur) {
            cand.clear();
            if (router_.start(cur, goal_)) {  // re-plan from the element the car actually occupies
                engineStale_ = false;
                cand = router_.routeAhead();
            }
        }
        if (cand.empty()) {
            event("NO_ROUTE", "engine=" + router_.state() + ";from=" + net_.elements[static_cast<size_t>(cur)].name);
            break;
        }
        const DrivePath cp(net_, cand);
        if (entersKnownClosure(cp, offset)) {
            event("ROUTE_REJECTED", "the engine route continues into a known closure on the occupied element (N2)");
            break;
        }
        const double aReq = requiredDecel(cp, speedCaps(cp), offset, v);
        const std::string cls = aReq <= vp_.decelComfort ? "COMFORT" : (aReq <= vp_.decelFirm ? "FIRM" : "INFEASIBLE");
        if (variant_ == Variant::B1 && cls == "INFEASIBLE") {
            int conn = -1;
            for (size_t k = 0; k < cand.size() && conn < 0; ++k) {
                if ((k >= old.size() || cand[k] != old[k]) && net_.elements[static_cast<size_t>(cand[k])].isConnector()) {
                    conn = cand[k];
                }
            }
            if (conn < 0) {
                event("COMMITMENT", "infeasible route with no diverging connector");
                break;
            }
            router_.reportClosed({conn});  // commitment mask: an engine obstacle, recorded apart from closures
            masks_.push_back({conn, net_.elements[static_cast<size_t>(conn)].fromLane});
            event("MASK_SET", "connector=" + net_.elements[static_cast<size_t>(conn)].name + ";a_req=" +
                                  (std::isfinite(aReq) ? fmt(aReq) : std::string("inf")) + ";v=" + fmt(v));
            continue;
        }
        adopt(cp, offset, reason);
        mode_ = Mode::Drive;
        stopActive_ = false;
        stopLatched_ = false;
        stopClass_.clear();
        std::string via;
        for (int e : cand) {
            if (net_.elements[static_cast<size_t>(e)].isConnector()) {
                via += (via.empty() ? "" : " ") + net_.elements[static_cast<size_t>(e)].name;
            }
        }
        event("REPLAN", "reason=" + reason + ";class=" + cls + ";a_req=" + (std::isfinite(aReq) ? fmt(aReq) : std::string("inf")) +
                            ";v=" + fmt(v) + ";via=" + via);
        return;
    }
    planClosureStop(reason, v);
}

void Stack::recover(const std::string& reason) {
    const int cur = currentElement();
    std::vector<int> cand;
    if (router_.active() && !engineStale_ && router_.consistent()) {
        cand = router_.routeAhead();
    }
    if (cand.empty() || cand.front() != cur) {
        cand.clear();
        if (router_.start(cur, goal_)) {
            engineStale_ = false;
            cand = router_.routeAhead();
        }
    }
    if (cand.empty()) {
        event("RECOVERY_FAILED", "reason=" + reason + ";engine=" + router_.state());
        return;
    }
    const DrivePath cp(net_, cand);
    const double offset = s_ - path_.elementStart(idx_);
    const double aReq = requiredDecel(cp, speedCaps(cp), offset, v_);
    if (entersKnownClosure(cp, offset) || aReq > vp_.decelFirm) {
        event("RECOVERY_FAILED", "reason=" + reason + ";route not drivable");
        return;
    }
    adopt(cp, offset, reason);
    mode_ = Mode::Drive;
    stopActive_ = false;
    stopLatched_ = false;
    stopClass_.clear();
    stopClosure_.clear();
    event("RECOVERED", "reason=" + reason + ";elements=" + std::to_string(cand.size()));
}

void Stack::planClosureStop(const std::string& reason, double v) {
    const double sb = nearestBarrierAhead(path_, s_);
    if (sb < 0.0) {
        fallback("no route and no closure ahead: " + reason);
        return;
    }
    for (const ClosureReport& c : known_) {
        if (barrierS(path_, c) == sb) {
            stopClosure_ = c.id;
        }
    }
    const double front = s_ + vp_.frontOffset();
    const double target = sb - kStopMargin - vp_.frontOffset();  // rear-axle s for a 2 m gap
    const double aReq = stopDecel(target - s_, v, a_, vp_.commandDelay, vp_.longLag);
    std::string cls;
    if (aReq <= vp_.decelComfort) {
        cls = "COMFORT";
        stopS_ = target;
        stopLimit_ = vp_.decelFirm;
        stopLatched_ = aReq >= kEngage;
    } else if (aReq <= vp_.decelFirm) {
        cls = "FIRM";
        stopS_ = target;
        stopLimit_ = kFirmHold;
        stopLatched_ = true;
    } else {
        const double a0 = stopDecel(sb - vp_.frontOffset() - s_, v, a_, vp_.commandDelay, vp_.longLag);
        cls = a0 <= vp_.decelEmergency ? "EMERGENCY" : "UNAVOIDABLE";
        stopS_ = sb - vp_.frontOffset();
        stopLimit_ = vp_.decelEmergency;
        stopLatched_ = true;
    }
    stopActive_ = true;
    stopClass_ = cls;
    mode_ = Mode::StopForClosure;
    event("STOP_PLANNED", "class=" + cls + ";a_req=" + (std::isfinite(aReq) ? fmt(aReq) : std::string("inf")) + ";v=" + fmt(v, 3) + ";a=" + fmt(a_, 3) +
                              ";front_to_barrier=" + fmt(sb - front, 3) + ";reason=" + reason);
}

void Stack::fallback(const std::string& reason) {
    event("MONITOR_TRIGGER", reason);
    event("FALLBACK_STOP", "v=" + fmt(v_) + ";decel=" + fmt(vp_.decelEmergency));
    mode_ = Mode::Fallback;
    stopActive_ = false;
}

void Stack::computeEnvelope() {
    env_.clear();
    if (path_.empty()) {
        return;
    }
    const std::vector<PathSample>& smp = path_.samples();
    const size_t i0 = path_.sampleIndexAt(s_);
    size_t i1 = i0;
    while (i1 + 1 < smp.size() && smp[i1 + 1].s <= s_ + kHorizon) {
        ++i1;
    }
    env_.assign(i1 - i0 + 1, 0.0);
    envFrom_ = i0;
    env_.back() = (i1 + 1 == smp.size()) ? 0.0 : caps_[i1];  // the end of the path is a stop
    for (size_t i = i1; i-- > i0;) {
        const double next = env_[i + 1 - i0];
        env_[i - i0] = std::min(caps_[i], std::sqrt(next * next + 2.0 * aEnv_ * (smp[i + 1].s - smp[i].s)));
    }
}

double Stack::envelopeAt(double s) const {
    if (env_.empty()) {
        return 0.0;
    }
    const std::vector<PathSample>& smp = path_.samples();
    if (s >= path_.length()) {
        return 0.0;
    }
    size_t i = std::max(path_.sampleIndexAt(s), envFrom_);
    const size_t last = envFrom_ + env_.size() - 1;
    if (i >= last) {
        return env_.back();
    }
    const double a = env_[i - envFrom_], b = env_[i + 1 - envFrom_];
    const double span = smp[i + 1].s - smp[i].s;
    const double t = span > 0.0 ? std::min(1.0, std::max(0.0, (s - smp[i].s) / span)) : 0.0;
    return a + (b - a) * t;
}

Command Stack::control(const VehicleState& pose, double t) {
    t_ = t;
    track(pose);
    Command cmd;
    if (path_.empty()) {
        cmd.accel = -kHoldDecel;
        return cmd;
    }

    // Steering: pure pursuit from the pose predicted over the command delay and part of the steering lag.
    const double L = vp_.wheelbase;
    const double v = pose.v;
    const double dyaw = v * std::tan(pose.steer) / L * kSteerPredict;
    const double xp = pose.x + v * kSteerPredict * std::cos(pose.yaw + 0.5 * dyaw);
    const double yp = pose.y + v * kSteerPredict * std::sin(pose.yaw + 0.5 * dyaw);
    const double yawp = pose.yaw + dyaw;
    const double look = std::min(kLookMax, kLookBase + kLookGain * v);
    const double sTarget = s_ + v * kSteerPredict + look;
    PathSample tg = path_.at(sTarget);
    const double over = sTarget - path_.length();
    if (over > 0.0) {
        tg.x += over * std::cos(tg.yaw);
        tg.y += over * std::sin(tg.yaw);
    }
    const double dx = tg.x - xp, dy = tg.y - yp;
    const double lx = std::cos(yawp) * dx + std::sin(yawp) * dy;
    const double ly = -std::sin(yawp) * dx + std::cos(yawp) * dy;
    const double l2 = lx * lx + ly * ly;
    const double kappa = l2 > 1e-6 ? 2.0 * ly / l2 : 0.0;
    cmd.steer = std::max(-vp_.maxSteer, std::min(vp_.maxSteer, std::atan(L * kappa)));

    // Longitudinal.
    switch (mode_) {
        case Mode::Fallback:
            cmd.accel = -vp_.decelEmergency;
            return cmd;
        case Mode::HoldFallback:
        case Mode::Hold:
        case Mode::Done:
        case Mode::NoRoute:
            cmd.accel = -kHoldDecel;
            return cmd;
        case Mode::LocalisationStop:  // stop in lane, steering on the estimate, and wait for it to recover
            cmd.accel = v < 0.05 ? -kHoldDecel : -locpolicy::kStopDecel;
            return cmd;
        default:
            break;
    }
    const double sp = s_ + v * kPreview;
    const double ve = envelopeAt(sp), ve2 = envelopeAt(sp + 1.0);
    const double aff = std::max(-aEnv_, std::min(0.0, 0.5 * (ve2 * ve2 - ve * ve)));
    double a = aff + kKp * (ve - v);
    if (speedCap_ < 1e8) {
        // Degraded localisation: also hold the capped speed, slowing to it gently. The envelope's own
        // braking (curves, stops) still applies on top.
        a = std::min(a, std::max(-1.5, kKp * (speedCap_ - v)));
    }
    double limit = vp_.decelFirm;
    auto stopLaw = [&](double target, bool& latched, double cap) {
        const double req = stopDecel(target - s_, v, pose.accel, vp_.commandDelay, vp_.longLag);
        if (!latched && req >= kEngage) {
            latched = true;
        }
        return latched ? -std::min(req, cap) : kInf;
    };
    if (goalOnPath_ && mode_ == Mode::Drive) {
        a = std::min(a, stopLaw(goalS_, goalLatched_, vp_.decelFirm));
    }
    if (stopActive_) {
        if (stopClass_ == "EMERGENCY" || stopClass_ == "UNAVOIDABLE") {
            a = -vp_.decelEmergency;
            limit = vp_.decelEmergency;
        } else {
            a = std::min(a, stopLaw(stopS_, stopLatched_, stopLimit_));
            limit = stopLimit_;
        }
    }
    a = std::max(-limit, std::min(vp_.accelComfort, a));
    const bool atGoal = goalLatched_ && goalS_ - s_ < 1.0;
    const bool atStop = stopActive_ && stopLatched_ && stopS_ - s_ < 1.0;
    if (v < 0.05 && (atGoal || atStop)) {
        a = -kHoldDecel;
    }
    cmd.accel = a;
    return cmd;
}

}  // namespace road

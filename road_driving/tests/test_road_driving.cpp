// Behavioural tests for road_driving. Each check states a property that could be false; none asserts
// that a function returns what its own code computes. Returns non-zero on any failure, as the engine's
// test programs do.
#include "road_driving/drive_path.hpp"
#include "road_driving/engine_router.hpp"
#include "road_driving/episode.hpp"
#include "road_driving/lane_network.hpp"
#include "road_driving/vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <random>
#include <string>
#include <vector>

using namespace road;

namespace {

int g_pass = 0, g_fail = 0;

void check(bool ok, const std::string& what, const std::string& detail = "") {
    if (ok) {
        ++g_pass;
    } else {
        ++g_fail;
        std::printf("FAIL  %s%s%s\n", what.c_str(), detail.empty() ? "" : "  -- ", detail.c_str());
    }
}

std::string f3(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.3f", v);
    return b;
}

// Reads key=value from an event detail.
double field(const std::string& detail, const std::string& key) {
    const size_t a = detail.find(key + "=");
    if (a == std::string::npos) {
        return std::nan("");
    }
    return std::atof(detail.c_str() + a + key.size() + 1);
}

std::string textField(const std::string& detail, const std::string& key) {
    const size_t a = detail.find(key + "=");
    if (a == std::string::npos) {
        return "";
    }
    const size_t b = detail.find(';', a);
    return detail.substr(a + key.size() + 1, b == std::string::npos ? std::string::npos : b - a - key.size() - 1);
}

const LogEvent* findEvent(const EpisodeResult& r, const std::string& source, const std::string& type) {
    for (const LogEvent& e : r.events) {
        if (e.source == source && e.type == type) {
            return &e;
        }
    }
    return nullptr;
}

bool visits(const LaneNetwork& net, const EpisodeResult& r, const std::string& name) {
    for (int id : r.metrics.visited) {
        if (net.elements[static_cast<size_t>(id)].name == name) {
            return true;
        }
    }
    return false;
}

Scenario named(const std::string& name, Variant v = Variant::B1, std::uint64_t seed = 0) {
    Scenario s;
    if (!findScenario(name, s)) {
        std::printf("FAIL  unknown scenario %s\n", name.c_str());
        ++g_fail;
    }
    s.variant = v;
    s.seed = seed;
    return s;
}

// ---- T1 map invariants ------------------------------------------------------------------------------
void testMap() {
    const LaneNetwork net = buildR1();
    const std::vector<std::string> errs = net.validate(1.0);
    check(errs.empty(), "T1 R1 passes every map invariant", errs.empty() ? "" : errs.front());
    check(net.validate(10.0).empty(), "T1 R1 stays inside the engine coordinate bound at 10 graph units per metre");
    check(!mirrored(net).validate().empty(), "T1 a mirrored (right-hand traffic) copy of R1 is rejected");
    check(net.connector(net.lane("J10>J11"), net.lane("J11>J21")) < 0, "T1 the banned right turn at J11 has no connector");
    check(net.lane("J21>J11") < 0 && net.lane("J11>J01") < 0, "T1 the one-way street has no westbound lane");
    const Lane& east = net.lanes[static_cast<size_t>(net.lane("J00>J10"))];
    check(std::fabs(east.y0 - 1.5) < 1e-12 && std::fabs(east.y1 - 1.5) < 1e-12, "T1 the eastbound lane runs on the north (left) side of its street");
    int uTurns = 0, turnsChecked = 0;
    for (const Element& e : net.elements) {
        if (!e.isConnector()) {
            continue;
        }
        const Lane& a = net.lanes[static_cast<size_t>(e.fromLane)];
        const Lane& b = net.lanes[static_cast<size_t>(e.toLane)];
        if (a.from == b.to && a.to == b.from) {
            ++uTurns;
        }
        // Left in left-hand traffic is the tight near-side turn: a counter-clockwise heading change.
        const double turned = wrapAngle(e.pts.back().yaw - e.pts.front().yaw);
        const bool okTurn = (e.turn == Turn::Left && turned > 1.5) || (e.turn == Turn::Right && turned < -1.5) ||
                            (e.turn == Turn::Ahead && std::fabs(turned) < 1e-9);
        turnsChecked += okTurn ? 1 : 0;
        check(okTurn, "T1 connector " + e.name + " turns the way its label says");
    }
    check(uTurns == 0, "T1 no connector is a U-turn", std::to_string(uTurns));
    const Lane& cul = net.lanes[static_cast<size_t>(net.lane("J21>D21"))];
    check(net.elements[static_cast<size_t>(cul.segments.back())].next.empty(), "T1 the cul-de-sac has no turning head (dead end)");
}

// ---- T2, T3 engine adapter against an independent reference -----------------------------------------
void testEngine() {
    const LaneNetwork net = buildR1();
    const int n = static_cast<int>(net.elements.size());
    std::mt19937_64 rng(20261116);
    int queries = 0, mismatchFound = 0, mismatchCost = 0, illegal = 0, throughBlocked = 0, unitDiffer = 0, reachable = 0;
    for (int q = 0; q < 400 && queries < 200; ++q) {
        const int s = static_cast<int>(rng() % static_cast<unsigned>(n));
        const int g = static_cast<int>(rng() % static_cast<unsigned>(n));
        if (s == g) {
            continue;
        }
        ++queries;
        std::vector<int> blocked;
        if (queries % 2 == 0) {
            while (blocked.size() < 3) {
                const int b = static_cast<int>(rng() % static_cast<unsigned>(n));
                if (b != s && b != g && std::find(blocked.begin(), blocked.end(), b) == blocked.end()) {
                    blocked.push_back(b);
                }
            }
        }
        EngineRouter router(net), router10(net, 10.0);
        router.reportClosed(blocked);
        router10.reportClosed(blocked);
        const bool ok = router.start(s, g);
        const bool ok10 = router10.start(s, g);
        const RefRoute ref = referenceRoute(net, s, g, blocked);
        mismatchFound += (ok != ref.found) ? 1 : 0;
        if (ok && ref.found) {
            ++reachable;
            const std::vector<int> route = router.routeAhead();
            illegal += routeLegal(net, route) ? 0 : 1;
            mismatchCost += std::fabs(routeCost(net, route) - ref.cost) < 1e-9 ? 0 : 1;
            for (int b : blocked) {
                throughBlocked += std::find(route.begin(), route.end(), b) != route.end() ? 1 : 0;
            }
            unitDiffer += (ok10 && router10.routeAhead() == route) ? 0 : 1;
        } else {
            unitDiffer += (ok10 != ok) ? 1 : 0;
        }
    }
    const std::string d = std::to_string(queries) + " queries, " + std::to_string(reachable) + " reachable";
    check(queries == 200 && mismatchFound == 0, "T2 engine and independent Dijkstra agree on reachability", d);
    check(mismatchCost == 0, "T2 every engine route costs exactly the independent optimum", std::to_string(mismatchCost) + " differ");
    check(illegal == 0, "T2 every engine route uses only legal moves", std::to_string(illegal) + " illegal");
    check(throughBlocked == 0, "T2 no engine route passes a reported closure", std::to_string(throughBlocked));
    check(unitDiffer == 0, "T3 routes are identical at 1 and 10 graph units per metre", std::to_string(unitDiffer) + " differ");
    check(reachable > 100, "T2 the corpus exercises mostly reachable pairs", d);
}

// ---- T4 vehicle model -------------------------------------------------------------------------------
void testVehicle() {
    VehicleParams ideal;
    ideal.commandDelay = 0.0;
    ideal.steerLag = 0.0;
    ideal.longLag = 0.0;
    ideal.maxSteerRate = 100.0;
    {
        VehicleState s0;
        s0.v = 5.0;
        s0.steer = 0.3;
        Vehicle veh(ideal, 0.01, s0);
        double travelled = 0.0, turned = 0.0;
        for (int k = 0; k < 2000; ++k) {
            const double yaw0 = veh.state().yaw;
            veh.step({0.3, 0.0});
            travelled += 5.0 * 0.01;
            turned += wrapAngle(veh.state().yaw - yaw0);
        }
        const double expected = ideal.wheelbase / std::tan(0.3);
        const double measured = travelled / turned;
        check(std::fabs(measured / expected - 1.0) < 0.01, "T4 a held steering angle gives radius wheelbase/tan(angle) within 1%",
              f3(measured) + " vs " + f3(expected));
    }
    {
        VehicleState s0;
        s0.v = 20.0 * kMph;
        Vehicle veh(ideal, 0.01, s0);
        double x0 = veh.state().x;
        for (int k = 0; k < 2000 && veh.state().v > 0.0; ++k) {
            veh.step({0.0, -3.0});
        }
        const double expected = s0.v * s0.v / (2.0 * 3.0);
        const double measured = veh.state().x - x0;
        check(std::fabs(measured / expected - 1.0) < 0.01, "T4 constant deceleration stops in v^2/(2a) within 1%", f3(measured) + " vs " + f3(expected));
    }
    {
        VehicleParams p;
        VehicleState s0;
        s0.v = 5.0;
        Vehicle veh(p, 0.01, s0);
        double maxRate = 0.0;
        bool movedEarly = false;
        for (int k = 0; k < 300; ++k) {
            veh.step({0.5, 1.0});
            maxRate = std::max(maxRate, std::fabs(veh.lastSteerRate()));
            if (k < 10 && (veh.state().steer != 0.0 || veh.state().accel != 0.0)) {
                movedEarly = true;
            }
        }
        check(maxRate <= p.maxSteerRate + 1e-9, "T4 a steering step never exceeds the rate limit", f3(maxRate));
        check(!movedEarly, "T4 a command takes effect only after the 0.10 s delay");
        check(std::fabs(veh.state().steer - 0.5) < 1e-3, "T4 the steering reaches a commanded angle inside the limit");
        Vehicle veh2(p, 0.01, s0);
        for (int k = 0; k < 400; ++k) {
            veh2.step({2.0, 0.0});
        }
        check(veh2.state().steer <= p.maxSteer + 1e-12, "T4 the steering angle never exceeds 0.55 rad");
    }
}

// ---- T5 path builder --------------------------------------------------------------------------------
void testPaths() {
    const LaneNetwork net = buildR1();
    const int start = net.lanes[static_cast<size_t>(net.lane("J00>J10"))].segments.front();
    const int goal = net.segmentAt(net.lane("J21>J22"), 47.5);
    const int closed = net.segmentAt(net.lane("J20>J21"), 27.5);
    for (const std::vector<int>& blocked : {std::vector<int>{}, std::vector<int>{closed}}) {
        const RefRoute ref = referenceRoute(net, start, goal, blocked);
        const DrivePath p(net, ref.nodes);
        double worstJump = 0.0, worstGap = 0.0, worstK = 0.0;
        const auto& smp = p.samples();
        for (size_t i = 1; i < smp.size(); ++i) {
            const double ds = smp[i].s - smp[i - 1].s;
            worstJump = std::max(worstJump, std::fabs(smp[i].kappa - smp[i - 1].kappa) - (1.0 / 7.5 / 5.0) * ds);
            worstGap = std::max(worstGap, std::hypot(smp[i].x - smp[i - 1].x, smp[i].y - smp[i - 1].y) - ds);
            worstK = std::max(worstK, std::fabs(smp[i].kappa));
        }
        const std::string which = blocked.empty() ? "direct route" : "detour around X1";
        check(ref.found && worstJump <= 1e-9, "T5 curvature is continuous along the " + which, f3(worstJump));
        check(worstGap <= 1e-6, "T5 consecutive samples are joined without gaps along the " + which, f3(worstGap));
        check(worstK <= 0.20, "T5 curvature never exceeds 0.20 1/m along the " + which, f3(worstK));
        const Element& g = net.elements[static_cast<size_t>(goal)];
        check(std::hypot(smp.back().x - g.pts.back().x, smp.back().y - g.pts.back().y) < 1e-9 &&
                  std::fabs(wrapAngle(smp.back().yaw - g.pts.back().yaw)) < 1e-9,
              "T5 the path ends on the goal lane's own geometry (" + which + ")");
    }
}

// ---- T6 nominal drive, T7 legal direction ------------------------------------------------------------
void checkDriveQuality(const EpisodeResult& r, const std::string& tag) {
    const Metrics& m = r.metrics;
    check(m.maxCrossTrackLink <= 0.20, tag + ": cross-track on links <= 0.20 m", f3(m.maxCrossTrackLink));
    check(m.maxCrossTrackConnector <= 0.30, tag + ": cross-track on connectors <= 0.30 m", f3(m.maxCrossTrackConnector));
    check(m.maxSteer <= 0.55 + 1e-9, tag + ": steering within 0.55 rad", f3(m.maxSteer));
    check(m.maxLatAccel <= 2.2, tag + ": lateral acceleration <= 2.2 m/s^2", f3(m.maxLatAccel));
    check(m.maxSpeedExcess <= 0.2, tag + ": never more than 0.2 m/s over the limit", f3(m.maxSpeedExcess));
    check(m.illegalTransitions == 0 && m.offLaneSamples == 0, tag + ": no illegal move and no off-lane sample",
          std::to_string(m.illegalTransitions) + "/" + std::to_string(m.offLaneSamples));
}

void testNominalAndLegal() {
    for (std::uint64_t seed = 0; seed <= 5; ++seed) {
        const EpisodeResult r = runEpisode(named("c0_nominal", Variant::B1, seed));
        const std::string tag = "T6 c0 seed " + std::to_string(seed);
        check(r.outcome == "COMPLETED", tag + ": completes", r.outcome);
        check(r.metrics.maxDecel <= 2.0, tag + ": braking stays within the 2.0 m/s^2 comfort limit", f3(r.metrics.maxDecel));
        checkDriveQuality(r, tag);
    }
    const LaneNetwork net = buildR1();
    std::mt19937_64 rng(7);
    int driven = 0;
    for (int attempt = 0; attempt < 200 && driven < 12; ++attempt) {
        const Lane& a = net.lanes[rng() % net.lanes.size()];
        const Lane& b = net.lanes[rng() % net.lanes.size()];
        if (a.name == b.name) {
            continue;
        }
        const int s = a.segments.front(), g = net.segmentAt(net.lane(b.name), 0.5 * b.length);
        if (!referenceRoute(net, s, g).found) {
            continue;
        }
        Scenario sc;
        sc.name = "t7_" + a.name + "_to_" + b.name;
        sc.startLane = a.name;
        sc.startLaneS = 5.0;
        sc.goalLane = b.name;
        sc.goalLaneS = 0.5 * b.length;
        const EpisodeResult r = runEpisode(sc);
        ++driven;
        check(r.outcome == "COMPLETED", "T7 " + sc.name + " completes", r.outcome);
        checkDriveQuality(r, "T7 " + sc.name);
    }
    check(driven == 12, "T7 twelve random legal missions were driven closed-loop", std::to_string(driven));
}

// ---- C1, C2: prior and advance notification follow the best route under full knowledge ---------------
void testPriorAndAdvance() {
    const LaneNetwork net = buildR1();
    const int start = net.lanes[static_cast<size_t>(net.lane("J00>J10"))].segments.front();
    const int goal = net.segmentAt(net.lane("J21>J22"), 47.5);
    std::vector<int> closed;
    for (int id : net.lanes[static_cast<size_t>(net.lane("J20>J21"))].segments) {
        const Element& e = net.elements[static_cast<size_t>(id)];
        if (e.laneS0 < 42.5 && e.laneS0 + e.length > 27.5) {
            closed.push_back(id);
        }
    }
    const RefRoute best = referenceRoute(net, start, goal, closed);
    for (const char* name : {"c1_prior", "c2_advance"}) {
        const EpisodeResult r = runEpisode(named(name));
        check(r.outcome == "COMPLETED", std::string("A6 ") + name + " completes", r.outcome);
        std::vector<int> connectorsDriven, connectorsBest;
        for (int id : r.metrics.visited) {
            if (net.elements[static_cast<size_t>(id)].isConnector()) {
                connectorsDriven.push_back(id);
            }
        }
        for (int id : best.nodes) {
            if (net.elements[static_cast<size_t>(id)].isConnector()) {
                connectorsBest.push_back(id);
            }
        }
        check(connectorsDriven == connectorsBest, std::string("A6 ") + name + " drives the best legal route under full knowledge",
              routeText(net, connectorsDriven));
        checkDriveQuality(r, std::string("A6 ") + name);
    }
}

// ---- T8 late closures: stopping and avoidability -----------------------------------------------------
// Stopping distance under full braking from speed v0 and current acceleration a0: the acceleration
// holds for the command delay, then follows a first-order lag towards -A. Closed form; stop time by bisection.
double analyticStop(double v0, double a0, double A, double delay, double tau) {
    const double v1 = v0 + a0 * delay;
    const double d1 = v0 * delay + 0.5 * a0 * delay * delay;
    auto vel = [&](double t) { return v1 - A * t + (a0 + A) * tau * (1.0 - std::exp(-t / tau)); };
    double lo = 0.0, hi = 20.0;
    for (int i = 0; i < 100; ++i) {
        const double mid = 0.5 * (lo + hi);
        (vel(mid) > 0.0 ? lo : hi) = mid;
    }
    const double T = 0.5 * (lo + hi);
    return d1 + v1 * T - 0.5 * A * T * T + (a0 + A) * tau * (T - tau * (1.0 - std::exp(-T / tau)));
}

const TruthSample* sampleAt(const EpisodeResult& r, double t) {
    for (const TruthSample& s : r.truth) {
        if (std::fabs(s.t - t) < 1e-9) {
            return &s;
        }
    }
    return nullptr;
}

// The constant deceleration command that stops within d, found numerically from analyticStop (an
// independent route to what the stack computes in closed form).
double requiredDecelNumeric(double d, double v, double a0) {
    const VehicleParams vp;
    double lo = 1e-3, hi = 100.0;
    if (analyticStop(v, a0, hi, vp.commandDelay, vp.longLag) > d) {
        return hi;
    }
    for (int i = 0; i < 100; ++i) {
        const double mid = 0.5 * (lo + hi);
        (analyticStop(v, a0, mid, vp.commandDelay, vp.longLag) > d ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

// Stop class by hand, and whether the decision sits within `tol` of a class boundary (where a different
// floating-point library could legitimately pick the neighbouring class).
std::string analyticClass(double v, double a0, double frontToBarrier, bool* nearBoundary = nullptr) {
    const double tol = 0.05;
    const double a = requiredDecelNumeric(frontToBarrier - stackparams::kStopMargin, v, a0);
    const double a0full = requiredDecelNumeric(frontToBarrier, v, a0);
    if (nearBoundary != nullptr) {
        *nearBoundary = std::fabs(a - 2.0) < tol || std::fabs(a - 3.5) < tol || (a > 3.5 && std::fabs(a0full - 8.0) < tol);
    }
    if (a <= 2.0) return "COMFORT";
    if (a <= 3.5) return "FIRM";
    return a0full <= 8.0 ? "EMERGENCY" : "UNAVOIDABLE";
}

void testLateClosures() {
    const VehicleParams vp;
    for (int range : {80, 60, 45, 30, 20, 12, 6}) {
        const std::string name = "c3_r" + std::to_string(range);
        const EpisodeResult r = runEpisode(named(name));
        const LogEvent* stop = findEvent(r, "STACK", "STOP_PLANNED");
        check(stop != nullptr, "T8 " + name + ": the stack plans a stop (no legal continuation exists)");
        if (stop == nullptr) {
            continue;
        }
        const double v = field(stop->detail, "v"), a0 = field(stop->detail, "a"), ftb = field(stop->detail, "front_to_barrier");
        const std::string cls = textField(stop->detail, "class");
        bool nearBoundary = false;
        const std::string hand = analyticClass(v, a0, ftb, &nearBoundary);
        check(cls == hand || nearBoundary, "T8 " + name + ": stop class matches the hand calculation",
              cls + " vs " + hand + " at v=" + f3(v) + " a=" + f3(a0) + " range=" + f3(ftb));
        const Metrics& m = r.metrics;
        // Travel of the front bumper from the decision to standstill, measured from truth.
        const double travel = m.incursion ? ftb + m.incursionDepth : ftb - m.stopMargin;
        if (cls == "COMFORT" || cls == "FIRM") {
            check(r.outcome == "STOPPED_AT_CLOSURE" && m.stopMargin >= 1.0, "A7 " + name + ": stops with at least 1.0 m to the barrier",
                  r.outcome + " margin " + f3(m.stopMargin));
            check(std::fabs(m.stopMargin - stackparams::kStopMargin) <= 0.5, "T8 " + name + ": stop margin within 0.5 m of the planned 2.0 m",
                  f3(m.stopMargin));
            if (cls == "COMFORT") {
                check(m.maxDecel <= 2.0, "T8 " + name + ": a comfort-class stop stays within the 2.0 m/s^2 comfort limit", f3(m.maxDecel));
            } else {
                check(m.maxDecel <= 3.9, "A7 " + name + ": deceleration at most 3.9 m/s^2", f3(m.maxDecel));
            }
        } else {
            // Analytic stop under full braking with this model, from the truth state at the decision.
            const TruthSample* at = sampleAt(r, stop->t);
            const double predicted = at ? analyticStop(at->v, at->accel, vp.decelEmergency, vp.commandDelay, vp.longLag) : -1.0;
            check(at != nullptr && std::fabs(travel - predicted) <= 0.5,
                  "T8 " + name + ": braking distance agrees with the analytic value within 0.5 m", f3(travel) + " vs " + f3(predicted));
            check(m.emergencyEvents >= 1, "A7 " + name + ": the emergency braking is flagged");
            if (cls == "EMERGENCY") {
                check(!m.incursion && r.outcome == "STOPPED_AT_CLOSURE", "A7 " + name + ": emergency braking avoids incursion", r.outcome);
            } else {
                check(r.outcome == "CLOSURE_INCURSION", "A7 " + name + ": unavoidable case is labelled and its depth measured",
                      r.outcome + " depth " + f3(m.incursionDepth));
            }
        }
    }
}

// ---- T9 commitment at J10 ---------------------------------------------------------------------------
void testCommitment() {
    const LaneNetwork net = buildR1();
    const VehicleParams vp;
    const Element& left = net.elements[static_cast<size_t>(net.connector(net.lane("J00>J10"), net.lane("J10>J11")))];
    const double stopLineX = net.lanes[static_cast<size_t>(net.lane("J00>J10"))].x1;
    for (int d : {40, 25, 15, 8, 3}) {
        const std::string name = "c4_d" + std::to_string(d);
        const EpisodeResult r = runEpisode(named(name));
        const LogEvent* obs = findEvent(r, "STACK", "CLOSURE_OBSERVED");
        check(obs != nullptr, "T9 " + name + ": the closure is observed");
        if (obs == nullptr) {
            continue;
        }
        const TruthSample* at = nullptr;
        for (const TruthSample& s : r.truth) {
            if (std::fabs(s.t - obs->t) < 1e-9) {
                at = &s;
            }
        }
        check(at != nullptr, "T9 " + name + ": a truth sample exists at the decision time");
        if (at == nullptr) {
            continue;
        }
        // Hand calculation from the map and the truth state: the deceleration needed to meet the left
        // connector's curvature speed caps after the latency budget.
        const double toConnector = stopLineX - at->x;
        double need = 0.0;
        for (const PathPoint& p : left.pts) {
            double cap = left.speedLimit;
            if (std::fabs(p.kappa) > 1e-9) {
                cap = std::min(cap, std::sqrt(stackparams::kLatPlan / std::fabs(p.kappa)));
            }
            cap += stackparams::kCapTolerance;
            if (at->v <= cap) {
                continue;
            }
            const double dist = toConnector + p.s - at->v * stackparams::kLatency;
            need = std::max(need, dist > 0.05 ? (at->v * at->v - cap * cap) / (2.0 * dist) : 1e9);
        }
        const bool expectLeft = need <= vp.decelFirm;
        const bool tookLeft = visits(net, r, "J10:W>N");
        check(tookLeft == expectLeft, "T9 " + name + ": branch at J10 matches the slow-for-turn calculation",
              std::string(tookLeft ? "left" : "ahead") + ", needed " + f3(need) + " m/s^2");
        if (expectLeft) {
            check(r.outcome == "COMPLETED", "T9 " + name + ": completes on the detour", r.outcome);
        } else {
            check(r.outcome == "STOPPED_AT_CLOSURE" && r.metrics.masks >= 1, "T9 " + name + ": masks the turn and stops before the barrier",
                  r.outcome);
        }
    }
    // Variant B0 on the same late notification accepts the engine's detour it cannot slow for.
    const EpisodeResult b0 = runEpisode(named("c4_d3", Variant::B0));
    check(b0.stackFinalMode == "HOLD_AFTER_FALLBACK" && b0.outcome == "STOPPED_EN_ROUTE" && b0.metrics.fallbacks == 1,
          "B0 c4_d3 ends in the monitor's fallback stop (the stored failing episode)", b0.outcome + " / " + b0.stackFinalMode);
    check(!b0.metrics.incursion, "B0 c4_d3 fallback stop does not enter the closure");
}

// ---- T10 closure lifecycle --------------------------------------------------------------------------
void testLifecycle() {
    {
        Scenario sc = named("c3_r20");
        sc.name = "c3_r20_coarse";
        sc.maxSegment = 60.0;  // the closed lane is one segment, so the car occupies the closed node
        const EpisodeResult r = runEpisode(sc);
        const LogEvent* report = findEvent(r, "STACK", "ENGINE_REPORT");
        check(report != nullptr && report->detail == "state=ACTIVE", "T10 the engine ignores a closure on the node the car occupies (N2)",
              report ? report->detail : "no report");
        check(findEvent(r, "STACK", "ROUTE_REJECTED") != nullptr, "T10 the stack rejects the engine route that runs into it");
        check(r.outcome == "STOPPED_AT_CLOSURE" && !r.metrics.incursion, "T10 and still stops before the barrier",
              r.outcome + " margin " + f3(r.metrics.stopMargin));
    }
    {
        const EpisodeResult r = runEpisode(named("c5_lift"));
        check(findEvent(r, "WORLD", "CLOSURE_LIFTED") != nullptr && findEvent(r, "STACK", "RECOVERED") != nullptr,
              "T10 a lift while held at the closure leads to a recovered route");
        check(r.outcome == "COMPLETED" && !r.metrics.incursion, "A8 c5_lift completes after the lift", r.outcome);
    }
    {
        const EpisodeResult r = runEpisode(named("c4_d3"));
        const LogEvent* set = findEvent(r, "STACK", "MASK_SET");
        const LogEvent* cleared = findEvent(r, "STACK", "MASK_CLEARED");
        check(set != nullptr && cleared != nullptr && cleared->t > set->t, "T10 a commitment mask is cleared after its junction");
    }
}

// ---- T11 determinism, replay and evaluator independence ----------------------------------------------
void testDeterminismAndReplay() {
    int differ = 0;
    for (const Scenario& base : catalogue()) {
        const std::uint64_t h1 = runEpisode(base).traceHash;
        const std::uint64_t h2 = runEpisode(base).traceHash;
        const std::uint64_t h3 = runEpisode(base).traceHash;
        if (h1 != h2 || h2 != h3) {
            ++differ;
            std::printf("      %s gives different traces\n", base.name.c_str());
        }
    }
    check(differ == 0, "T11 three runs of every catalogue scenario give identical trace hashes", std::to_string(differ));

    const EpisodeResult r = runEpisode(named("c4_d3", Variant::B0, 3));
    const std::string dir = (std::filesystem::temp_directory_path() / "road_driving_replay_test").string();
    std::string error;
    check(writeEpisode(r, dir, error), "replay: the episode folder is written", error);
    std::map<std::string, std::string> kv;
    Scenario sc;
    const bool read = readKeyValues(dir + "/manifest.txt", kv, error) && scenarioFromManifest(kv, sc, error);
    check(read, "replay: the manifest reads back into a scenario", error);
    const EpisodeResult again = runEpisode(sc);
    check(hex64(again.traceHash) == kv["trace_hash"] && again.outcome == kv["outcome"], "replay: the manifest reproduces the trace and the outcome");
    std::string stored;
    check(readText(dir + "/truth.csv", stored) && stored == again.truthCsv, "replay: the stored truth.csv is reproduced byte for byte");
    sc.revealDistance += 0.5;
    check(runEpisode(sc).traceHash != r.traceHash, "replay: a changed scenario input changes the trace hash (the check can fail)");
    std::filesystem::remove_all(dir);

    // Evaluator independence: a hand-written truth log that drives through the barrier is flagged.
    const LaneNetwork net = buildR1();
    const VehicleParams vp;
    Scenario x1 = named("c3_r45");
    std::vector<TruthSample> fake;
    for (int i = 0; i <= 400; ++i) {
        const double y = 12.5 + 0.1 * i;  // rear axle along the closed northbound lane J20>J21
        fake.push_back({0.02 * i, 238.5, y, kPi / 2.0, i == 400 ? 0.0 : 5.0, 0.0, 0.0, 0.0, 0.0, 2});
    }
    std::string outcome;
    const Metrics m = evaluate(net, vp, x1, fake, -1.0, outcome);
    check(m.incursion && outcome == "CLOSURE_INCURSION", "T13 the evaluator flags an incursion from truth alone", outcome);
    check(std::fabs(m.incursionDepth - (52.5 - 40.0 + vp.frontOffset())) < 0.05, "T13 and measures its depth",
          f3(m.incursionDepth));
    x1.closure = false;
    evaluate(net, vp, x1, fake, -1.0, outcome);
    check(outcome != "CLOSURE_INCURSION", "T13 the same log without a closure is not an incursion", outcome);
}

}  // namespace

int main() {
    testMap();
    testEngine();
    testVehicle();
    testPaths();
    testNominalAndLegal();
    testPriorAndAdvance();
    testLateClosures();
    testCommitment();
    testLifecycle();
    testDeterminismAndReplay();
    std::printf("road_driving tests: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

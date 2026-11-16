// Independent evaluation: everything here is computed from the truth samples, the scenario and the lane
// network. The stack's status, routes and events are not read.
#include "road_driving/episode.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace road {

namespace {

double yawAt(const Element& e, double s) {
    const auto it = std::upper_bound(e.pts.begin(), e.pts.end(), s, [](double v, const PathPoint& p) { return v < p.s; });
    if (it == e.pts.begin()) {
        return e.pts.front().yaw;
    }
    if (it == e.pts.end()) {
        return e.pts.back().yaw;
    }
    const PathPoint& b = *it;
    const PathPoint& a = *(it - 1);
    const double t = b.s > a.s ? (s - a.s) / (b.s - a.s) : 0.0;
    return wrapAngle(a.yaw + wrapAngle(b.yaw - a.yaw) * t);
}

bool siblings(const LaneNetwork& net, int a, int b) {
    for (int p : net.elements[static_cast<size_t>(a)].prev) {
        const std::vector<int>& nx = net.elements[static_cast<size_t>(p)].next;
        if (std::find(nx.begin(), nx.end(), b) != nx.end()) {
            return true;
        }
    }
    return false;
}

}  // namespace

Metrics evaluate(const LaneNetwork& net, const VehicleParams& vp, const Scenario& sc, const std::vector<TruthSample>& truth,
                 double tLift, std::string& outcome) {
    Metrics m;
    if (truth.empty()) {
        outcome = "TIMEOUT";
        return m;
    }
    const double inf = std::numeric_limits<double>::infinity();
    const double half = 0.5 * vp.widthMirrors;
    const double fo = vp.frontOffset();
    const PathPoint goal = net.lanePoint(net.lane(sc.goalLane), sc.goalLaneS);
    const int closureLane = sc.closure ? net.lane(sc.closureLane) : -1;
    const PathPoint barrier = closureLane >= 0 ? net.lanePoint(closureLane, sc.barrierLaneS) : PathPoint{};
    const double bdx = std::cos(barrier.yaw), bdy = std::sin(barrier.yaw);
    auto present = [&](double t) { return closureLane >= 0 && (tLift < 0.0 || t < tLift); };
    // Furthest progress of a front corner past the barrier line, over corners inside the closed lane's band.
    auto progress = [&](const TruthSample& s, bool& inBand) {
        const double fx = s.x + fo * std::cos(s.yaw), fy = s.y + fo * std::sin(s.yaw);
        const double nx = -std::sin(s.yaw), ny = std::cos(s.yaw);
        double best = -inf;
        inBand = false;
        for (double side : {-1.0, 1.0}) {
            const double cx = fx + side * half * nx - barrier.x, cy = fy + side * half * ny - barrier.y;
            const double p = cx * bdx + cy * bdy;
            const double q = -cx * bdy + cy * bdx;
            if (std::fabs(q) <= 0.5 * net.spec.laneWidth + 0.5) {
                inBand = true;
                best = std::max(best, p);
            }
        }
        return best;
    };

    int cur = associate(net, truth[0].x, truth[0].y, truth[0].yaw);
    m.visited.push_back(cur);
    m.maxSpeedExcess = -inf;
    size_t limited = 0, pairs = 0;
    bool emergency = false;
    for (size_t i = 0; i < truth.size(); ++i) {
        const TruthSample& s = truth[i];
        // Lane association restricted to legal continuations: the current element, two successors deep,
        // and its siblings (the other movements of the same approach, which overlap at a junction mouth).
        std::vector<int> cand = {cur};
        for (int n : net.elements[static_cast<size_t>(cur)].next) {
            cand.push_back(n);
            for (int n2 : net.elements[static_cast<size_t>(n)].next) {
                cand.push_back(n2);
            }
        }
        for (int p : net.elements[static_cast<size_t>(cur)].prev) {
            for (int sib : net.elements[static_cast<size_t>(p)].next) {
                cand.push_back(sib);
            }
        }
        int best = -1;
        double bestD = inf, bestLat = 0.0;
        for (int c : cand) {
            double es = 0.0, lat = 0.0;
            const double d = distanceToElement(net.elements[static_cast<size_t>(c)], s.x, s.y, &es, &lat);
            if (std::fabs(wrapAngle(yawAt(net.elements[static_cast<size_t>(c)], es) - s.yaw)) > kPi / 3.0) {
                continue;
            }
            if (d < bestD - 1e-9) {
                bestD = d;
                best = c;
                bestLat = lat;
            }
        }
        if (best >= 0 && bestD <= 1.5) {
            if (best != cur) {
                if (siblings(net, cur, best)) {
                    m.visited.back() = best;
                } else {
                    m.visited.push_back(best);
                }
                cur = best;
            }
            double& slot = net.elements[static_cast<size_t>(cur)].isConnector() ? m.maxCrossTrackConnector : m.maxCrossTrackLink;
            slot = std::max(slot, std::fabs(bestLat));
        } else {
            ++m.offLaneSamples;
            double dg = inf;
            const int g = associate(net, s.x, s.y, s.yaw, &dg);
            if (g >= 0 && dg <= 1.0 && std::find(cand.begin(), cand.end(), g) == cand.end()) {
                ++m.illegalTransitions;
                m.visited.push_back(g);
                cur = g;
            }
        }

        m.maxSpeedExcess = std::max(m.maxSpeedExcess, s.v - net.elements[static_cast<size_t>(cur)].speedLimit);
        m.maxSteer = std::max(m.maxSteer, std::fabs(s.steer));
        m.maxLatAccel = std::max(m.maxLatAccel, s.v * s.v * std::fabs(std::tan(s.steer)) / vp.wheelbase);
        if (s.v > 0.05) {
            // At rest the model's acceleration state is the brake-hold command, not a deceleration.
            m.maxDecel = std::max(m.maxDecel, -s.accel);
        }
        m.maxAccel = std::max(m.maxAccel, s.accel);
        if (i > 0) {
            const double dt = s.t - truth[i - 1].t;
            m.maxJerk = std::max(m.maxJerk, std::fabs(s.accel - truth[i - 1].accel) / dt);
            if (std::fabs(s.steer - truth[i - 1].steer) / dt >= vp.maxSteerRate - 1e-3) {
                ++limited;
            }
            ++pairs;
        }
        const bool e = -s.accel > stackparams::kEmergencyEvent;
        if (e && !emergency) {
            ++m.emergencyEvents;
        }
        emergency = e;
        if (present(s.t)) {
            bool inBand = false;
            const double p = progress(s, inBand);
            if (inBand && p > 0.0 && p < sc.worksLength + 10.0) {
                m.incursion = true;
                m.incursionDepth = std::max(m.incursionDepth, p);
            }
        }
    }
    m.steerRateLimitedShare = pairs > 0 ? static_cast<double>(limited) / static_cast<double>(pairs) : 0.0;

    const TruthSample& last = truth.back();
    const bool atRest = last.v < 0.05;
    const double gx = last.x - goal.x, gy = last.y - goal.y;
    const double along = gx * std::cos(goal.yaw) + gy * std::sin(goal.yaw);
    const double lat = -gx * std::sin(goal.yaw) + gy * std::cos(goal.yaw);
    m.completed = atRest && std::fabs(along) <= 1.0 && std::fabs(lat) <= 0.5;
    if (m.completed) {
        m.timeToGoal = last.t;
    }
    if (atRest && present(last.t)) {
        bool inBand = false;
        const double p = progress(last, inBand);
        if (inBand && p <= 0.0 && p > -30.0) {
            m.stopMargin = -p;
        }
    }
    if (m.incursion) {
        outcome = "CLOSURE_INCURSION";
    } else if (m.completed) {
        outcome = "COMPLETED";
    } else if (atRest && m.stopMargin >= 0.0) {
        outcome = "STOPPED_AT_CLOSURE";
    } else if (atRest) {
        outcome = "STOPPED_EN_ROUTE";
    } else {
        outcome = "TIMEOUT";
    }
    return m;
}

}  // namespace road

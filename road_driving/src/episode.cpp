#include "road_driving/episode.hpp"

#include "road_driving/drive_path.hpp"
#include "stack.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>

#ifndef ROAD_ENGINE_COMMIT
#define ROAD_ENGINE_COMMIT "unknown"
#endif
#ifndef ROAD_ENGINE_SOURCES_MODIFIED
#define ROAD_ENGINE_SOURCES_MODIFIED -1
#endif

namespace road {

namespace {

struct SplitMix64 {
    std::uint64_t state;
    std::uint64_t next() {
        std::uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
    double uniform(double a, double b) { return a + (b - a) * (static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0)); }
};

std::string exact(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.17g", v);
    return buf;
}

std::string fixed(double v, int prec) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*f", prec, v);
    return buf;
}

}  // namespace

const char* variantName(Variant v) { return v == Variant::B0 ? "B0" : "B1"; }

bool parseVariant(const std::string& s, Variant& v) {
    if (s == "B0") {
        v = Variant::B0;
        return true;
    }
    if (s == "B1") {
        v = Variant::B1;
        return true;
    }
    return false;
}

const char* revealName(Reveal r) {
    switch (r) {
        case Reveal::AtStart: return "at_start";
        case Reveal::BeforeJunction: return "before_junction";
        case Reveal::BeforeBarrier: return "before_barrier";
        default: return "none";
    }
}

bool parseReveal(const std::string& s, Reveal& r) {
    for (Reveal c : {Reveal::None, Reveal::AtStart, Reveal::BeforeJunction, Reveal::BeforeBarrier}) {
        if (s == revealName(c)) {
            r = c;
            return true;
        }
    }
    return false;
}

std::vector<Scenario> catalogue() {
    std::vector<Scenario> out;
    Scenario s;
    s.name = "c0_nominal";
    s.scenarioClass = "C0";
    out.push_back(s);
    Scenario cl;
    cl.closure = true;
    s = cl;
    s.name = "c1_prior";
    s.scenarioClass = "C1";
    s.reveal = Reveal::AtStart;
    out.push_back(s);
    s = cl;
    s.name = "c2_advance";
    s.scenarioClass = "C2";
    s.reveal = Reveal::BeforeJunction;
    s.revealDistance = 60.0;
    out.push_back(s);
    for (int r : {80, 60, 45, 30, 20, 12, 6}) {
        s = cl;
        s.name = "c3_r" + std::to_string(r);
        s.scenarioClass = "C3";
        s.reveal = Reveal::BeforeBarrier;
        s.revealDistance = r;
        out.push_back(s);
    }
    for (int d : {40, 25, 15, 8, 3}) {
        s = cl;
        s.name = "c4_d" + std::to_string(d);
        s.scenarioClass = "C4";
        s.reveal = Reveal::BeforeJunction;
        s.revealDistance = d;
        out.push_back(s);
    }
    s = cl;
    s.name = "c5_lift";
    s.scenarioClass = "C5";
    s.reveal = Reveal::BeforeBarrier;
    s.revealDistance = 45.0;
    s.liftAfterStop = 30.0;
    out.push_back(s);
    return out;
}

bool findScenario(const std::string& name, Scenario& out) {
    for (const Scenario& s : catalogue()) {
        if (s.name == name) {
            out = s;
            return true;
        }
    }
    return false;
}

std::uint64_t fnv1a(const std::string& text) {
    std::uint64_t h = 0xcbf29ce484222325ULL;
    for (unsigned char c : text) {
        h ^= c;
        h *= 0x100000001b3ULL;
    }
    return h;
}

std::string hex64(std::uint64_t v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "0x%016llx", static_cast<unsigned long long>(v));
    return buf;
}

std::string toolchain() {
#if defined(__clang__)
    return std::string("Clang ") + __clang_version__;
#elif defined(_MSC_VER)
    return "MSVC " + std::to_string(_MSC_FULL_VER);
#elif defined(__GNUC__)
    return std::string("GCC ") + __VERSION__;
#else
    return "unknown";
#endif
}

std::string engineCommit() { return ROAD_ENGINE_COMMIT; }

bool engineSourcesModified() { return ROAD_ENGINE_SOURCES_MODIFIED == 1; }

EpisodeResult runEpisode(const Scenario& sc) {
    EpisodeResult r;
    r.scenario = sc;
    NetworkSpec ns;
    ns.maxSegment = sc.maxSegment;
    const LaneNetwork net = buildNetwork(sc.network, ns);
    const std::vector<std::string> errs = net.validate();
    if (!errs.empty()) {
        throw std::runtime_error("network " + sc.network + " fails validation: " + errs.front());
    }
    const bool estimatedMode = sc.mode == "estimated";
    if (!estimatedMode && sc.mode != "ground_truth") {
        throw std::runtime_error("unknown mode " + sc.mode);
    }
    if (estimatedMode && sc.condition.empty()) {
        throw std::runtime_error("estimated mode needs a sensor condition");
    }
    const VehicleParams vp;
    const double dt = 0.01;
    const int startLane = net.lane(sc.startLane), goalLane = net.lane(sc.goalLane);
    const int closureLane = sc.closure ? net.lane(sc.closureLane) : -1;
    if (startLane < 0 || goalLane < 0 || (sc.closure && closureLane < 0)) {
        throw std::runtime_error("scenario " + sc.name + " names an unknown lane");
    }

    // Seeded jitter of the start pose and, for C3 and C4, of the notification distance.
    if (sc.seed != 0) {
        SplitMix64 rng{sc.seed};
        r.jitterAlong = rng.uniform(-2.0, 2.0);
        r.jitterLateral = rng.uniform(-0.2, 0.2);
        r.jitterHeading = rng.uniform(-2.0, 2.0) * kPi / 180.0;
        const double jr = rng.uniform(-1.0, 1.0);
        if (sc.scenarioClass == "C3" || sc.scenarioClass == "C4") {
            r.jitterReveal = jr;
        }
    }
    const PathPoint sp = net.lanePoint(startLane, sc.startLaneS + r.jitterAlong);
    VehicleState init;
    init.x = sp.x - std::sin(sp.yaw) * r.jitterLateral;
    init.y = sp.y + std::cos(sp.yaw) * r.jitterLateral;
    init.yaw = wrapAngle(sp.yaw + r.jitterHeading);
    const int startElem = associate(net, init.x, init.y, init.yaw);
    const int goalElem = net.segmentAt(goalLane, sc.goalLaneS);
    if (startElem < 0) {
        throw std::runtime_error("start pose is on no lane");
    }

    // Closure X1 as the world knows it: present from the start; the stack learns of it only when notified.
    std::vector<int> closed;
    if (sc.closure) {
        for (int id : net.lanes[static_cast<size_t>(closureLane)].segments) {
            const Element& e = net.elements[static_cast<size_t>(id)];
            if (e.laneS0 < sc.barrierLaneS + sc.worksLength && e.laneS0 + e.length > sc.barrierLaneS) {
                closed.push_back(id);
            }
        }
    }

    // The oracle measures notification distances along the route the car takes when nothing is closed,
    // from the world's own reference search (not the engine's route).
    const RefRoute ref = referenceRoute(net, startElem, goalElem);
    const DrivePath corridor(net, ref.nodes);
    double junctionS = -1.0, barrierS = -1.0;
    const int rj = net.junction(sc.revealJunction);
    for (size_t i = 0; i < ref.nodes.size(); ++i) {
        const Element& e = net.elements[static_cast<size_t>(ref.nodes[i])];
        if (junctionS < 0.0 && e.isConnector() && e.junction == rj) {
            junctionS = corridor.elementStart(i);
        }
        if (sc.closure && e.id == net.segmentAt(closureLane, sc.barrierLaneS)) {
            barrierS = corridor.elementStart(i) + (sc.barrierLaneS - e.laneS0);
        }
    }

    // Sensors (world side) and, in estimated mode, the estimator that drives. Condition spans along the
    // route become map-frame rectangles around the route elements they cover.
    std::unique_ptr<SensorSimulator> sensors;
    std::unique_ptr<Estimator> est;
    if (!sc.condition.empty()) {
        const SensorConditions cond = sensorConditions(sc.condition);
        const double L = corridor.length();
        auto spanZones = [&](const Span& span, std::vector<Zone>& out) {
            if (!span.active()) {
                return;
            }
            const double s0 = span.from * L, s1 = span.to * L;
            const size_t n = corridor.elements().size();
            for (size_t i = 0; i < n; ++i) {
                const double a = std::max(corridor.elementStart(i), s0);
                const double b = std::min(i + 1 < n ? corridor.elementStart(i + 1) : L, s1);
                if (b <= a) {
                    continue;
                }
                Zone z{1e18, 1e18, -1e18, -1e18};
                for (double s = a;; s = std::min(b, s + 0.5)) {
                    const PathSample p = corridor.at(s);
                    z.x0 = std::min(z.x0, p.x - 4.0);
                    z.y0 = std::min(z.y0, p.y - 4.0);
                    z.x1 = std::max(z.x1, p.x + 4.0);
                    z.y1 = std::max(z.y1, p.y + 4.0);
                    if (s >= b) {
                        break;
                    }
                }
                out.push_back(z);
            }
        };
        spanZones(cond.outage, r.zones.outage);
        spanZones(cond.multipath, r.zones.multipath);
        spanZones(cond.slip, r.zones.slip);
        spanZones(cond.ramp, r.zones.ramp);
        spanZones(cond.kerbBlackout, r.zones.kerbBlackout);
        spanZones(cond.kerbShift, r.zones.kerbShift);
        if (cond.parked.active()) {
            // Parked cars against the opposite kerb along the route's lanes (not in junctions): unmapped
            // objects for the kerb scanner, outside the car's own lane.
            const double s0 = cond.parked.from * L, s1 = cond.parked.to * L;
            for (double s = s0; s + 4.5 <= s1; s += 7.0) {
                const PathSample p = corridor.at(s + 2.25);
                const PathSample q = corridor.at(s + 4.5);
                const PathSample a = corridor.at(s);
                if (net.elements[static_cast<size_t>(p.element)].isConnector() || net.elements[static_cast<size_t>(q.element)].isConnector() ||
                    net.elements[static_cast<size_t>(a.element)].isConnector()) {
                    continue;
                }
                const double off = -(net.spec.laneWidth + 0.5 * net.spec.laneWidth - 1.0);  // car centre 0.1 m off the far kerb
                r.zones.obstacles.push_back({p.x - std::sin(p.yaw) * off, p.y + std::cos(p.yaw) * off, p.yaw, 4.5, 1.8});
            }
        }
        sensors.reset(new SensorSimulator(cond, r.zones, sc.seed, vp.wheelbase, &net));
        if (estimatedMode) {
            est = makeEstimator(sc.estimator, EstimatorConfig{}, net);
            if (!est) {
                throw std::runtime_error("unknown estimator " + sc.estimator);
            }
        }
    }

    Vehicle veh(vp, dt, init);
    Stack stack(net, vp, sc.variant, goalElem, sc.goalLaneS, sc.feasibility);
    bool present = sc.closure, revealed = false, lifted = false;
    double tRest = -1.0;
    std::vector<LogEvent> world;
    const std::string labels =
        estimatedMode ? "SYNTHETIC_ROADS,ESTIMATED_POSE(" + sc.estimator + "),ORACLE_CLOSURE" : "SYNTHETIC_ROADS,GROUND_TRUTH_POSE,ORACLE_CLOSURE";
    world.push_back({0.0, "WORLD", "EPISODE_START",
                     "network=" + sc.network + ";labels=" + labels + ";start=" + net.elements[static_cast<size_t>(startElem)].name +
                         ";goal=" + net.elements[static_cast<size_t>(goalElem)].name +
                         (sc.condition.empty() ? std::string() : ";sensors=" + sc.condition)});
    if (sc.closure && sc.reveal == Reveal::AtStart) {
        revealed = true;
        r.tReveal = 0.0;
        world.push_back({0.0, "WORLD", "CLOSURE_NOTIFIED", "id=X1;kind=ROADWORKS;source=ORACLE;when=before-departure"});
    }
    double sCorr = 0.0;
    {
        const DrivePath::Projection pc = corridor.project(init.x, init.y, 0.0, 0.0, net.elements[static_cast<size_t>(startElem)].length);
        sCorr = pc.ok ? pc.s : 0.0;
    }
    const double revealDist = sc.revealDistance + r.jitterReveal;

    std::string truth = "t,x,y,yaw,v,steer,accel,cmd_steer,cmd_accel,closure\n";
    std::string estimates = "t,x,y,yaw,v,pxx,pxy,pyy,pyawyaw,sigma_pos,status,gnss,kerb,shift\n";
    Command cmd;
    double restTime = 0.0, outageStart = -1.0, doneSince = -1.0;
    bool stackStarted = false;
    const long maxTicks = std::lround(sc.timeBudget / dt);
    for (long k = 0;; ++k) {
        const double t = static_cast<double>(k) * dt;
        const VehicleState vs = veh.state();

        // World: sensors; in estimated mode the estimator consumes what has arrived.
        StateEstimate estNow;
        if (sensors) {
            std::vector<Measurement> arrived;
            sensors->step(t, dt, vs.x, vs.y, vs.yaw, vs.v, vs.steer, vs.accel, arrived);
            if (est) {
                for (const Measurement& m : arrived) {
                    est->process(m);
                }
                estNow = est->estimateAt(t);
            }
            if (!sensors->gnssAvailable() && outageStart < 0.0) {
                outageStart = t;
            } else if (sensors->gnssAvailable() && outageStart >= 0.0) {
                r.outages.push_back({outageStart, t});
                outageStart = -1.0;
            }
        }
        // What the stack is given as its pose: simulator truth, or the estimate (never both).
        VehicleState stackPose = vs;
        if (est) {
            stackPose.x = estNow.x;
            stackPose.y = estNow.y;
            stackPose.yaw = estNow.yaw;
            stackPose.v = estNow.v;
            stackPose.steer = estNow.steer;
            stackPose.accel = estNow.accel;
        }
        const bool missionStarted = t >= sc.departDelay - 1e-9 && (!est || estNow.valid);

        // World: oracle notification and lift.
        const DrivePath::Projection pc = corridor.project(vs.x, vs.y, sCorr, 3.0, 10.0);
        const bool onCorridor = pc.ok && pc.distance < 2.0;
        if (onCorridor) {
            sCorr = pc.s;
        }
        if (sc.closure && present && !revealed && onCorridor) {
            const double front = sCorr + vp.frontOffset();
            const double toJ = junctionS - front, toB = barrierS - front;
            const bool fire = (sc.reveal == Reveal::BeforeJunction && junctionS >= 0.0 && toJ <= revealDist) ||
                              (sc.reveal == Reveal::BeforeBarrier && barrierS >= 0.0 && toB <= revealDist);
            if (fire) {
                revealed = true;
                r.tReveal = t;
                world.push_back({t, "WORLD", "CLOSURE_NOTIFIED",
                                 "id=X1;kind=ROADWORKS;source=ORACLE;front_to_junction=" + fixed(toJ, 3) + ";front_to_barrier=" + fixed(toB, 3) +
                                     ";v=" + fixed(vs.v, 3)});
            }
        }
        if (present && revealed && sc.liftAfterStop >= 0.0) {
            if (vs.v < 0.01) {
                if (tRest < 0.0) {
                    tRest = t;
                }
                if (t - tRest >= sc.liftAfterStop - 1e-9) {
                    present = false;
                    lifted = true;
                    r.tLift = t;
                    world.push_back({t, "WORLD", "CLOSURE_LIFTED", "id=X1"});
                }
            } else {
                tRest = -1.0;
            }
        }

        // Stack: planning at 10 Hz, control at 50 Hz.
        if (k % 10 == 0) {
            Observation obs;
            obs.t = t;
            obs.pose = stackPose;
            if (est) {
                obs.estimated = true;
                obs.estimateValid = estNow.valid && estNow.hasCov;  // quality unknown without a covariance: not usable
                obs.estimateAge = t - estNow.t;
                obs.sigmaAlong = estNow.sigmaAlong(estNow.yaw);
                obs.sigmaAcross = estNow.sigmaAcross(estNow.yaw);
                obs.sigmaHeading = std::isfinite(estNow.pyawyaw) && estNow.pyawyaw > 0.0 ? std::sqrt(estNow.pyawyaw) : 0.0;
            }
            if (sc.closure && revealed) {
                ClosureReport c;
                c.id = "X1";
                c.kind = "ROADWORKS";
                c.lane = closureLane;
                c.barrierLaneS = sc.barrierLaneS;
                c.worksLength = sc.worksLength;
                c.closed = closed;
                c.active = present;
                obs.closures.push_back(c);
            }
            if (!stackStarted) {
                if (missionStarted) {
                    stack.initialise(obs);
                    stackStarted = true;
                }
            } else {
                stack.plan(obs);
            }
        }
        if (k % 2 == 0) {
            cmd = stackStarted ? stack.control(stackPose, t) : Command{0.0, -2.0};  // brake hold until the mission starts
            r.belief.push_back({t, stackStarted ? stack.believedElement() : -1});
            if (est) {
                r.estimates.push_back(estNow);
                char eline[320];
                std::snprintf(eline, sizeof eline, "%.2f,%.4f,%.4f,%.5f,%.4f,%.6g,%.6g,%.6g,%.6g,%.4f,%s,%s,%s,%.3f\n", t, estNow.x, estNow.y,
                              estNow.yaw, estNow.v, estNow.pxx, estNow.pxy, estNow.pyy, estNow.pyawyaw, estNow.valid ? estNow.sigmaPos() : -1.0,
                              estNow.status, estNow.gnss, estNow.kerb, stackStarted ? stack.currentShift() : 0.0);
                estimates += eline;
            }
            const int code = !sc.closure ? 0 : (lifted ? 3 : (revealed ? 2 : 1));
            r.truth.push_back({t, vs.x, vs.y, vs.yaw, vs.v, vs.steer, vs.accel, cmd.steer, cmd.accel, code});
            char line[256];
            std::snprintf(line, sizeof line, "%.2f,%.6f,%.6f,%.7f,%.6f,%.7f,%.6f,%.7f,%.6f,%d\n", t, vs.x, vs.y, vs.yaw, vs.v, vs.steer, vs.accel,
                          cmd.steer, cmd.accel, code);
            truth += line;
        }

        restTime = vs.v < 0.01 ? restTime + dt : 0.0;
        // In estimated mode the stack's "done" rests on its estimate: the world waits for the true car to stop.
        if (stack.mode() == Stack::Mode::Done && doneSince < 0.0) {
            doneSince = t;
        }
        const bool done = stack.mode() == Stack::Mode::Done && (!estimatedMode || vs.v < 0.01 || t - doneSince >= 3.0);
        if (done || (stack.mode() == Stack::Mode::HoldFallback && restTime >= 3.0) || restTime >= 45.0 || k >= maxTicks) {
            r.simSeconds = t;
            break;
        }
        veh.step(cmd);
    }

    if (outageStart >= 0.0) {
        r.outages.push_back({outageStart, r.simSeconds});
    }
    if (sensors) {
        r.sensors = sensors->log();
        r.sensorsCsv = sensorLogCsv(r.sensors);
    }
    if (est) {
        r.estimatesCsv = estimates;
    }
    r.truthCsv = truth;
    r.traceHash = fnv1a(truth);
    r.stackFinalMode = stack.modeName();
    r.engineMs = stack.engineMs();
    r.engineSilencedBytes = stack.silencedBytes();
    r.engineNodes = stack.engineNodes();
    r.engineEdges = stack.engineEdges();

    r.events = world;
    r.events.insert(r.events.end(), stack.events.begin(), stack.events.end());
    std::stable_sort(r.events.begin(), r.events.end(), [](const LogEvent& a, const LogEvent& b) { return a.t < b.t; });
    r.eventsCsv = "t,source,type,detail\n";
    for (const LogEvent& e : r.events) {
        r.eventsCsv += fixed(e.t, 2) + "," + e.source + "," + e.type + "," + e.detail + "\n";
    }
    r.routes = stack.routes;
    r.routesCsv = "t,reason,engine_state,route\n";
    std::string decisions;
    for (const RouteRecord& rr : r.routes) {
        const std::string names = routeText(net, rr.elements);
        r.routesCsv += fixed(rr.t, 2) + "," + rr.reason + "," + rr.engineState + "," + names + "\n";
        decisions += rr.reason + "|" + names + "\n";
    }
    r.routeHash = fnv1a(decisions);

    // Independent evaluation from truth, then the stack's own decision counts kept apart.
    r.metrics = evaluate(net, vp, sc, r.truth, r.tLift, r.outcome);
    for (const LogEvent& e : stack.events) {
        if (e.type == "REPLAN") {
            ++r.metrics.replans;
        } else if (e.type == "MASK_SET") {
            ++r.metrics.masks;
        } else if (e.type == "FALLBACK_STOP") {
            ++r.metrics.fallbacks;
        } else if (e.type == "MONITOR_TRIGGER") {
            ++r.metrics.monitorTriggers;
        } else if (e.type == "STOP_PLANNED") {
            const size_t a = e.detail.find("class=");
            r.metrics.stopClass = e.detail.substr(a + 6, e.detail.find(';', a) - a - 6);
        } else if (e.type == "LOCALISATION_UNRELIABLE") {
            ++r.metrics.localisationStops;
        } else if (e.type == "LOCALISATION_DEGRADED") {
            ++r.metrics.localisationDegraded;
        } else if (e.type == "NO_CLEAR_CONTINUATION") {
            ++r.metrics.clearanceStops;
        }
        // A stop decided on the estimate, judged against truth at the moment of the decision.
        if (estimatedMode && (e.type == "LOCALISATION_UNRELIABLE" || e.type == "NO_CLEAR_CONTINUATION") && !r.estimates.empty()) {
            const size_t i = std::min(r.estimates.size() - 1, static_cast<size_t>(std::max(0.0, std::round(e.t / (2.0 * dt)))));
            if (i < r.truth.size() && r.estimates[i].valid &&
                std::hypot(r.estimates[i].x - r.truth[i].x, r.estimates[i].y - r.truth[i].y) < kUnwarrantedError) {
                ++r.metrics.unwarrantedStops;
            }
        }
    }

    // Lane association: the element the stack believes it is in against the evaluator's association from
    // truth, sample by sample. Movements of one approach overlap at the junction mouth, and an element
    // boundary is ambiguous by the position error, so lane-level agreement is what is counted.
    auto compatible = [&](int b, int t) {
        const Element& eb = net.elements[static_cast<size_t>(b)];
        const Element& et = net.elements[static_cast<size_t>(t)];
        if (b == t || (!eb.isConnector() && !et.isConnector() && eb.lane == et.lane)) return true;
        if (eb.isConnector() && !et.isConnector() && (eb.fromLane == et.lane || eb.toLane == et.lane)) return true;
        if (et.isConnector() && !eb.isConnector() && (et.fromLane == eb.lane || et.toLane == eb.lane)) return true;
        return eb.isConnector() && et.isConnector() && eb.fromLane == et.fromLane;
    };
    for (size_t i = 0; i < r.belief.size() && i < r.metrics.truthElement.size(); ++i) {
        const int b = r.belief[i].second, t = r.metrics.truthElement[i];
        if (b >= 0 && t >= 0) {
            ++r.metrics.associationSamples;
            r.metrics.associationErrors += compatible(b, t) ? 0 : 1;
        }
    }
    // The driving estimate's error against truth (estimated mode).
    if (!r.estimates.empty()) {
        std::vector<double> err;
        double sum2 = 0.0;
        for (size_t i = 0; i < r.estimates.size() && i < r.truth.size(); ++i) {
            if (r.estimates[i].valid) {
                const double e = std::hypot(r.estimates[i].x - r.truth[i].x, r.estimates[i].y - r.truth[i].y);
                err.push_back(e);
                sum2 += e * e;
            }
        }
        if (!err.empty()) {
            std::sort(err.begin(), err.end());
            r.metrics.estRmse = std::sqrt(sum2 / static_cast<double>(err.size()));
            r.metrics.estP95 = err[std::min(err.size() - 1, static_cast<size_t>(0.95 * static_cast<double>(err.size())))];
            r.metrics.estMax = err.back();
        }
    }
    return r;
}

std::string manifestText(const EpisodeResult& r) {
    const Scenario& s = r.scenario;
    const Metrics& m = r.metrics;
    std::ostringstream o;
    o << "format=road-driving-episode/1\n"
      << "labels=SYNTHETIC_ROADS,GROUND_TRUTH_POSE,ORACLE_CLOSURE\n"
      << "network=R1 synthetic left-hand-traffic grid, 20 mph, no real geometry\n"
      << "vehicle=ipace_proxy_v1 kinematic bicycle (approximate I-PACE dimensions)\n"
      << "scenario=" << s.name << "\nclass=" << s.scenarioClass << "\nvariant=" << variantName(s.variant) << "\nseed=" << s.seed
      << "\nmax_segment=" << exact(s.maxSegment) << "\nstart_lane=" << s.startLane << "\nstart_lane_s=" << exact(s.startLaneS)
      << "\ngoal_lane=" << s.goalLane << "\ngoal_lane_s=" << exact(s.goalLaneS) << "\nclosure=" << (s.closure ? 1 : 0)
      << "\nclosure_lane=" << s.closureLane << "\nbarrier_lane_s=" << exact(s.barrierLaneS) << "\nworks_length=" << exact(s.worksLength)
      << "\nreveal=" << revealName(s.reveal) << "\nreveal_distance=" << exact(s.revealDistance) << "\nreveal_junction=" << s.revealJunction
      << "\nlift_after_stop=" << exact(s.liftAfterStop) << "\ntime_budget=" << exact(s.timeBudget) << "\njitter_along=" << exact(r.jitterAlong)
      << "\njitter_lateral=" << exact(r.jitterLateral) << "\njitter_heading=" << exact(r.jitterHeading) << "\njitter_reveal=" << exact(r.jitterReveal)
      << "\nengine_commit=" << engineCommit() << "\nengine_sources_modified_at_configure=" << (engineSourcesModified() ? "yes" : "no")
      << "\ntoolchain=" << toolchain() << "\noutcome=" << r.outcome << "\nstack_final_mode=" << r.stackFinalMode
      << "\ncompleted=" << (m.completed ? 1 : 0) << "\ntime_to_goal=" << fixed(m.timeToGoal, 2) << "\nincursion=" << (m.incursion ? 1 : 0)
      << "\nincursion_depth=" << fixed(m.incursionDepth, 3) << "\nstop_margin=" << fixed(m.stopMargin, 3) << "\nstop_class="
      << (m.stopClass.empty() ? "none" : m.stopClass) << "\nreplans=" << m.replans << "\nmasks=" << m.masks << "\nfallbacks=" << m.fallbacks
      << "\nmonitor_triggers=" << m.monitorTriggers << "\nmax_cross_track_link=" << fixed(m.maxCrossTrackLink, 3)
      << "\nmax_cross_track_connector=" << fixed(m.maxCrossTrackConnector, 3) << "\nillegal_transitions=" << m.illegalTransitions
      << "\noff_lane_samples=" << m.offLaneSamples << "\nmax_speed_excess=" << fixed(m.maxSpeedExcess, 3) << "\nmax_steer=" << fixed(m.maxSteer, 4)
      << "\nsteer_rate_limited_share=" << fixed(m.steerRateLimitedShare, 4) << "\nmax_lat_accel=" << fixed(m.maxLatAccel, 3)
      << "\nmax_decel=" << fixed(m.maxDecel, 3) << "\nmax_accel=" << fixed(m.maxAccel, 3) << "\nmax_jerk=" << fixed(m.maxJerk, 2)
      << "\nemergency_events=" << m.emergencyEvents << "\nt_notified=" << fixed(r.tReveal, 2) << "\nt_lift=" << fixed(r.tLift, 2)
      << "\nsim_seconds=" << fixed(r.simSeconds, 2) << "\ntruth_rows=" << r.truth.size() << "\ntrace_hash=" << hex64(r.traceHash)
      << "\nroute_hash=" << hex64(r.routeHash) << "\nengine_nodes=" << r.engineNodes << "\nengine_edges=" << r.engineEdges
      << "\nengine_stdout_bytes_silenced=" << r.engineSilencedBytes << "\nengine_ms_wall_clock_not_hashed=" << fixed(r.engineMs, 3) << "\n";
    // Localisation fields (absent from manifests written before they existed; read as defaults).
    o << "network_name=" << s.network << "\ncondition=" << (s.condition.empty() ? "none" : s.condition) << "\nmode=" << s.mode
      << "\nestimator=" << s.estimator << "\ndepart_delay=" << exact(s.departDelay) << "\nmin_clearance=" << fixed(m.minClearance, 3)
      << "\nclearance_violations=" << m.clearanceViolations << "\nassociation_errors=" << m.associationErrors
      << "\nassociation_samples=" << m.associationSamples << "\nlocalisation_stops=" << m.localisationStops
      << "\nlocalisation_degraded=" << m.localisationDegraded << "\nest_rmse=" << fixed(m.estRmse, 4) << "\nest_p95=" << fixed(m.estP95, 4)
      << "\nest_max=" << fixed(m.estMax, 4) << "\nfeasibility=" << (s.feasibility ? "on" : "off") << "\nmax_crossing=" << fixed(m.maxCrossing, 3)
      << "\nclearance_stops=" << m.clearanceStops << "\nunwarranted_stops=" << m.unwarrantedStops << "\ngnss_outages=" << r.outages.size()
      << "\noutage_windows=";
    for (size_t i = 0; i < r.outages.size(); ++i) {
        o << (i ? "," : "") << fixed(r.outages[i].first, 2) << ":" << fixed(r.outages[i].second, 2);
    }
    auto zones = [&](const char* key, const std::vector<Zone>& zs) {
        o << "\n" << key << "=";
        for (size_t i = 0; i < zs.size(); ++i) {
            o << (i ? ";" : "") << fixed(zs[i].x0, 2) << ":" << fixed(zs[i].y0, 2) << ":" << fixed(zs[i].x1, 2) << ":" << fixed(zs[i].y1, 2);
        }
    };
    zones("zones_outage", r.zones.outage);
    zones("zones_multipath", r.zones.multipath);
    zones("zones_slip", r.zones.slip);
    zones("zones_ramp", r.zones.ramp);
    zones("zones_kerb_blackout", r.zones.kerbBlackout);
    if (!r.zones.kerbShift.empty()) zones("zones_kerb_shift", r.zones.kerbShift);  // absent from older logs
    o << "\nobstacles=";
    for (size_t i = 0; i < r.zones.obstacles.size(); ++i) {
        const Obstacle& ob = r.zones.obstacles[i];
        o << (i ? ";" : "") << fixed(ob.cx, 2) << ":" << fixed(ob.cy, 2) << ":" << fixed(ob.yaw, 4) << ":" << fixed(ob.length, 2) << ":" << fixed(ob.width, 2);
    }
    o << "\n";
    return o.str();
}

bool writeEpisode(const EpisodeResult& r, const std::string& dir, std::string& error) {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
        error = "cannot create " + dir + ": " + ec.message();
        return false;
    }
    const std::pair<const char*, std::string> files[] = {{"manifest.txt", manifestText(r)}, {"truth.csv", r.truthCsv},
                                                         {"events.csv", r.eventsCsv},     {"routes.csv", r.routesCsv},
                                                         {"sensors.csv", r.sensorsCsv},   {"estimates.csv", r.estimatesCsv}};
    for (const auto& f : files) {
        if (f.second.empty()) {
            continue;  // no sensors, or no estimator in the loop
        }
        std::ofstream out(std::filesystem::path(dir) / f.first, std::ios::binary);
        out << f.second;
        if (!out) {
            error = std::string("cannot write ") + f.first;
            return false;
        }
    }
    return true;
}

bool readText(const std::string& path, std::string& text) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    text = ss.str();
    return true;
}

bool readKeyValues(const std::string& path, std::map<std::string, std::string>& kv, std::string& error) {
    std::string text;
    if (!readText(path, text)) {
        error = "cannot read " + path;
        return false;
    }
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        const size_t eq = line.find('=');
        if (eq != std::string::npos) {
            kv[line.substr(0, eq)] = line.substr(eq + 1);
        }
    }
    return true;
}

bool scenarioFromManifest(const std::map<std::string, std::string>& kv, Scenario& sc, std::string& error) {
    try {
        auto get = [&](const char* k) -> const std::string& {
            const auto it = kv.find(k);
            if (it == kv.end()) {
                throw std::runtime_error(std::string("manifest lacks ") + k);
            }
            return it->second;
        };
        if (get("format") != "road-driving-episode/1") {
            throw std::runtime_error("unknown manifest format");
        }
        sc.name = get("scenario");
        sc.scenarioClass = get("class");
        if (!parseVariant(get("variant"), sc.variant) || !parseReveal(get("reveal"), sc.reveal)) {
            throw std::runtime_error("bad variant or reveal");
        }
        sc.seed = std::stoull(get("seed"));
        sc.maxSegment = std::stod(get("max_segment"));
        sc.startLane = get("start_lane");
        sc.startLaneS = std::stod(get("start_lane_s"));
        sc.goalLane = get("goal_lane");
        sc.goalLaneS = std::stod(get("goal_lane_s"));
        sc.closure = get("closure") == "1";
        sc.closureLane = get("closure_lane");
        sc.barrierLaneS = std::stod(get("barrier_lane_s"));
        sc.worksLength = std::stod(get("works_length"));
        sc.revealDistance = std::stod(get("reveal_distance"));
        sc.revealJunction = get("reveal_junction");
        sc.liftAfterStop = std::stod(get("lift_after_stop"));
        sc.timeBudget = std::stod(get("time_budget"));
        auto optional = [&](const char* k, const std::string& fallback) {
            const auto it = kv.find(k);
            return it == kv.end() ? fallback : it->second;
        };
        sc.network = optional("network_name", "R1");
        sc.condition = optional("condition", "none");
        if (sc.condition == "none") {
            sc.condition.clear();
        }
        sc.mode = optional("mode", "ground_truth");
        sc.estimator = optional("estimator", "ekf");
        sc.departDelay = std::stod(optional("depart_delay", "0"));
        sc.feasibility = optional("feasibility", "off") == "on";  // older manifests: the stack before the layer existed
    } catch (const std::exception& ex) {
        error = ex.what();
        return false;
    }
    return true;
}

}  // namespace road

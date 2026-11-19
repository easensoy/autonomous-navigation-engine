// The London district networks (district.hpp): Kensington and Knightsbridge. Geometry, lane graph, turning paths
// the vehicle can follow, kerbs the scanner and the clearance checks share, the routes, and their episodes. The San
// Francisco network (right-hand traffic) has its own tests: test_sf.cpp.
#include "road_driving/benchmark.hpp"
#include "road_driving/district.hpp"
#include "road_driving/episode.hpp"
#include "road_driving/live.hpp"
#include "road_driving/vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

using namespace road;

namespace {

int g_pass = 0, g_fail = 0;

void check(bool ok, const std::string& what, const std::string& detail = "") {
    (ok ? g_pass : g_fail)++;
    std::printf("%s %s%s%s\n", ok ? "PASS" : "FAIL", what.c_str(), detail.empty() ? "" : ": ", detail.c_str());
}

std::string num(double v, int prec = 3) {
    char b[64];
    std::snprintf(b, sizeof b, "%.*f", prec, v);
    return b;
}

void geometry(const std::string& name) {
    const LaneNetwork net = buildNetwork(name);
    const std::vector<std::string> errs = net.validate();
    check(net.general && errs.empty(), name + " builds as a district network and validates", errs.empty() ? "" : errs.front());

    // Left-hand traffic: every two-way street has its lanes left of the centreline in their direction of travel.
    bool left = true;
    for (const Lane& l : net.lanes) {
        left = left && l.dirX * (l.y0 - l.centreY0) - l.dirY * (l.x0 - l.centreX0) > 0.0;
    }
    check(left, name + ": every lane keeps left of its street's centreline");

    // Turning paths the car can follow: continuous joins, curvature within the vehicle's limit (validate), and
    // the swept footprint with mirrors on the carriageway along every movement and lane segment.
    const VehicleParams vp;
    double worst = 1e9;
    std::string where;
    int movements = 0;
    for (const Element& e : net.elements) {
        movements += e.isConnector() ? 1 : 0;
        for (size_t i = 0; i < e.pts.size(); i += 3) {
            const PathPoint& p = e.pts[i];
            const double c = footprintClearance(net, p.x, p.y, p.yaw, vp.rearOverhang, vp.frontOffset(), vp.widthMirrors);
            if (c < worst) {
                worst = c;
                where = e.name;
            }
        }
    }
    check(worst > 0.10, name + ": the footprint with mirrors clears the kerb by more than 0.10 m along all " + std::to_string(movements) + " movements",
          "smallest " + num(worst) + " m at " + where);

    // The sign of the kerb distance (from the nearest kerb point's normal) agrees with a point-in-polygon test
    // on the closed kerb loops, on a 1.7 m grid over the district.
    const auto loops = carriagewayLoops(net, 0.005);
    double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
    for (const auto& lp : loops) {
        for (const auto& p : lp) {
            x0 = std::min(x0, p.first);
            y0 = std::min(y0, p.second);
            x1 = std::max(x1, p.first);
            y1 = std::max(y1, p.second);
        }
    }
    int samples = 0, disagree = 0;
    for (double x = x0 - 5.0; x <= x1 + 5.0; x += 1.7) {
        for (double y = y0 - 5.0; y <= y1 + 5.0; y += 1.7) {
            const double d = net.roadClearance(x, y);
            if (std::fabs(d) < 0.01) continue;  // on the kerb, within the arcs' sampling
            ++samples;
            disagree += (d > 0.0) != insideCarriageway(loops, x, y) ? 1 : 0;
        }
    }
    check(samples > 1000 && disagree == 0, name + ": the kerb distance's sign matches a point-in-polygon test",
          std::to_string(disagree) + " of " + std::to_string(samples) + " points disagree");

    // Patch edges bound the road for clearance but are not kerbs: the scanner sees nothing on them.
    int edges = 0, seen = 0, unbounded = 0;
    for (const KerbLine& k : net.kerbLines) {
        if (!k.boundary) continue;
        ++edges;
        const double mx = 0.5 * (k.ax + k.bx), my = 0.5 * (k.ay + k.by);
        const double sx = mx + 6.0 * k.nx, sy = my + 6.0 * k.ny;  // 6 m inside the carriageway, looking out
        const double r = net.rayToKerb(sx, sy, -k.nx, -k.ny, 25.0);
        seen += (r > 0.0 && std::fabs(r - 6.0) < 0.5) ? 1 : 0;
        unbounded += net.roadClearance(sx, sy) > 6.0 + 1e-6 ? 1 : 0;  // the edge still bounds the modelled road
    }
    check(edges > 0 && seen == 0 && unbounded == 0,
          name + ": its " + std::to_string(edges) + " patch edges bound the road but give the scanner no return");

    // The scene message carries the district's provenance, street names and patch edges.
    const std::string scene = sceneJson(net);
    check(scene.find("\"district\":{") != std::string::npos && scene.find("\"street_info\":[") != std::string::npos &&
              scene.find("\"kerb_boundary\":[") != std::string::npos && scene.find("OpenStreetMap") != std::string::npos,
          name + ": the scene message names the district, its source and its patch edges");
}

void rules() {
    const LaneNetwork k = buildNetwork("KEN");
    const LaneNetwork b = buildNetwork("KNB");
    // One-way streets: Hans Road and Basil Street have no lane against their direction.
    check(b.lane("HRJ>HR1") >= 0 && b.lane("HR1>HRJ") < 0 && b.lane("HBJ>HCB") >= 0 && b.lane("HCB>HBJ") < 0,
          "KNB: Hans Road and Basil Street are one-way, as OpenStreetMap records them");
    // The recorded turn restriction (OSM relation 9932431): no right turn from Hans Road (towards Hans Place)
    // into Basil Street; the left turn into Basil Street from the one-way part is a movement.
    check(b.connector(b.lane("HRS>HBJ"), b.lane("HBJ>HCB")) < 0 && b.connector(b.lane("HR2>HBJ"), b.lane("HBJ>HCB")) >= 0,
          "KNB: no right turn from Hans Road into Basil Street; the left turn exists");
    check(b.banned.size() == 1, "KNB: its one banned turn is listed");
    // Queen's Gate: a boulevard with a central reservation; no U-turn through its crossovers.
    check(k.connector(k.lane("QGN>QPC"), k.lane("QPC>QGN")) < 0 && k.connector(k.lane("QGT>QPC"), k.lane("QPC>QGT")) < 0,
          "KEN: no U-turn on Queen's Gate");
    bool median = false;
    for (const Street& s : k.streets) median = median || (s.name == "Queen's Gate" && s.median > 4.0);
    check(median, "KEN: Queen's Gate has its central reservation");
    // The grids are not district networks and keep their coarse clearance sign.
    check(!buildNetwork("R7").general && !buildNetwork("R1").general, "the R1 to R7 grids are unchanged in kind");
}

bool london(const RouteSpec& rs) { return rs.network == "KEN" || rs.network == "KNB"; }

void routes() {
    for (const RouteSpec& rs : districtRoutes()) {
        if (!london(rs)) continue;
        const LaneNetwork net = buildNetwork(rs.network);
        const int start = net.segmentAt(net.lane(rs.startLane), rs.startS), goal = net.segmentAt(net.lane(rs.goalLane), rs.goalS);
        const RefRoute rr = referenceRoute(net, start, goal);
        std::string moves;
        for (int n : rr.nodes) {
            const Element& e = net.elements[static_cast<size_t>(n)];
            if (e.isConnector() && e.turn != Turn::Ahead) moves += std::string(moves.empty() ? "" : ", ") + turnName(e.turn) + " " + e.name;
        }
        check(rr.found && routeLegal(net, rr.nodes), rs.name + ": the route is reachable and legal", moves);
    }
    // The planned turns are the ones the routes are described by.
    const LaneNetwork k = buildNetwork("KEN");
    const RouteSpec ka = districtRoutes()[0];
    const RefRoute r1 = referenceRoute(k, k.segmentAt(k.lane(ka.startLane), ka.startS), k.segmentAt(k.lane(ka.goalLane), ka.goalS));
    const std::string t1 = routeText(k, r1.nodes);
    check(t1.find("QGN:KGW>QPC") != std::string::npos && t1.find("QEP:QGT>EGS") != std::string::npos,
          "KEN-A: left into Queen's Gate, right into Elvaston Place");
    const LaneNetwork b = buildNetwork("KNB");
    const RouteSpec kb = districtRoutes()[1];
    const RefRoute r2 = referenceRoute(b, b.segmentAt(b.lane(kb.startLane), kb.startS), b.segmentAt(b.lane(kb.goalLane), kb.goalS));
    const std::string t2 = routeText(b, r2.nodes);
    check(t2.find("HRJ:BRM>HR1") != std::string::npos && t2.find("HBJ:HR2>HCB") != std::string::npos && t2.find("HCB:HBJ>HCE") != std::string::npos,
          "KNB-A: left into Hans Road, left into Basil Street, right into Hans Crescent");
}

void episodes() {
    for (const RouteSpec& rs : districtRoutes()) {
        if (!london(rs)) continue;
        Scenario sc = benchmarkScenario(rs, "", 0, "ground_truth");
        const EpisodeResult a = runEpisode(sc);
        check(a.outcome == "COMPLETED" && a.metrics.maxCrossing == 0.0 && a.metrics.illegalTransitions == 0,
              rs.name + " on simulator truth completes on the carriageway",
              a.outcome + ", " + num(a.simSeconds, 2) + " s, smallest clearance " + num(a.metrics.minClearance) + " m");
        const EpisodeResult b = runEpisode(sc);
        check(a.traceHash == b.traceHash && a.traceHash != 0, rs.name + ": the same episode twice gives the same trace", hex64(a.traceHash));
        check(a.events.front().detail.find("SOURCE_INFORMED_ROADS") != std::string::npos, rs.name + ": its episode is labelled source-informed");
        for (const char* cond : {"nominal_ldn", "gnss_outage_ldn", "multipath_ldn"}) {
            Scenario se = benchmarkScenario(rs, cond, 1, "estimated");
            se.estimator = "ekf_kerb2";
            const EpisodeResult e = runEpisode(se);
            // Recorded, not required to pass: the localisation scenes show what happens, failures included.
            std::printf("INFO %s %s seed 1 ekf_kerb2: %s at %.2f s, deepest crossing %.3f m, position RMSE %.3f m, max %.3f m\n", rs.name.c_str(), cond,
                        e.outcome.c_str(), e.simSeconds, e.metrics.maxCrossing, e.metrics.estRmse, e.metrics.estMax);
            if (std::string(cond) == "nominal_ldn") {
                check(e.outcome == "COMPLETED", rs.name + " drives on ekf_kerb2's estimate without faults", e.outcome);
            }
        }
    }
    Scenario parked = benchmarkScenario(districtRoutes()[0], "combined_r7", 1, "ground_truth");
    bool threw = false;
    try {
        runEpisode(parked);
    } catch (const std::exception&) {
        threw = true;
    }
    check(threw, "a parked-car condition is refused on a district network (placed for the grids' widths only)");
}

}  // namespace

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    for (const std::string n : {"KEN", "KNB"}) geometry(n);
    rules();
    routes();
    episodes();
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 && g_pass > 0 ? 0 : 1;
}

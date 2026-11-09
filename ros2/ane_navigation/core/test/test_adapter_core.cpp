// Unit tests for the ROS-independent adapter core: graph frame, roadmap loading and clearance,
// blockage mapping, the segment controller's limits and modes, and the mission executive's state
// transitions around this repository's NavigationCore.
#include "ane_core/blockage_mapper.hpp"
#include "ane_core/mission_executive.hpp"
#include "ane_core/roadmap.hpp"
#include "ane_core/segment_controller.hpp"
#include "check.hpp"
#include "kinematic_world.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

using namespace ane;
using ane::test::Case;

namespace {

constexpr double kPi = 3.14159265358979323846;

std::shared_ptr<const Roadmap> warehouse() {
    static std::shared_ptr<const Roadmap> map = std::make_shared<const Roadmap>(Roadmap::load(ANE_ROADMAP_FILE));
    return map;
}

int nodeAt(const Roadmap& map, double x, double y) {
    return map.nearestNode({x, y}, 1e-6);
}

std::vector<Rect> staticRects(const Roadmap& map) {
    std::vector<Rect> rects;
    for (const StaticObstacle& o : map.obstacles()) {
        rects.push_back(o.rect);
    }
    return rects;
}

bool expectThrow(const std::string& text, const std::string& fragment) {
    try {
        Roadmap::parse(text, "test.txt");
    } catch (const std::runtime_error& error) {
        return std::string(error.what()).find(fragment) != std::string::npos;
    }
    return false;
}

const std::string kMinimal =
    "ane-roadmap 1\nframe 1 0 0 0\nhall -5 -5 5 5\nnode 0 0 0\nnode 1 1 0\nedge 0 1 1\n";

// Feeds poses along the middle aisle (y = 0) from x0 to x1 and steps the executive at each one.
double driveAlongMiddle(MissionExecutive& exec, double x0, double x1, double t) {
    const int steps = static_cast<int>(std::ceil(std::abs(x1 - x0) / 0.02));
    for (int k = 1; k <= steps; ++k) {
        t += 0.05;
        const double x = x0 + (x1 - x0) * k / steps;
        exec.updatePose(t, {x, 0.0, x1 >= x0 ? 0.0 : kPi});
        exec.step(t, 0.05);
    }
    return t;
}

bool hasEvent(const std::vector<MissionEvent>& events, const std::string& type) {
    return std::any_of(events.begin(), events.end(), [&](const MissionEvent& e) { return e.type == type; });
}

}  // namespace

int main() {
    std::vector<Case> cases;

    cases.push_back({"graph frame converts both ways and rejects a bad scale", [] {
        const GraphFrame frame(0.1, 0.4, {-8.0, -5.0});
        std::mt19937_64 rng(7);
        std::uniform_real_distribution<double> u(-200.0, 200.0);
        for (int i = 0; i < 1000; ++i) {
            const Point2 g{u(rng), u(rng)};
            const Point2 back = frame.toGraph(frame.toMap(g));
            CHECK_NEAR(back.x, g.x, 1e-9);
            CHECK_NEAR(back.y, g.y, 1e-9);
        }
        const GraphFrame warehouseFrame(0.1, 0.0, {-8.0, -5.0});
        const Point2 m = warehouseFrame.toMap({15.0, 50.0});
        CHECK_NEAR(m.x, -6.5, 1e-12);
        CHECK_NEAR(m.y, 0.0, 1e-12);
        bool threw = false;
        try {
            GraphFrame(0.0, 0.0, {0, 0});
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);
    }});

    cases.push_back({"the committed warehouse roadmap loads with its stations and frame", [] {
        const Roadmap& map = *warehouse();
        CHECK(map.name() == "ane_warehouse");
        CHECK(map.graph().getNodeCount() == 89);
        CHECK(map.edges().size() == 90);
        CHECK(map.station("dispatch") == 0);
        CHECK(map.station("delivery") == 88);
        CHECK(map.station("nowhere") == -1);
        CHECK_NEAR(map.nodePosition(0).x, -6.5, 1e-9);
        CHECK_NEAR(map.nodePosition(88).x, 6.5, 1e-9);
        CHECK(map.graph().getNode(0).getName() == "dispatch");
        CHECK(map.obstacles().size() == 10);  // 4 walls, 4 racks, 2 tables
        CHECK(map.zones().size() == 1);
    }});

    cases.push_back({"malformed roadmaps are rejected with the file and line", [] {
        CHECK(expectThrow("frame 1 0 0 0\n", "test.txt:1"));
        CHECK(expectThrow(kMinimal + "bogus 1\n", "unknown keyword"));
        CHECK(expectThrow(kMinimal + "node 0 2 2\n", "duplicate node"));
        CHECK(expectThrow(kMinimal + "edge 0 7 1\n", "unknown node"));
        CHECK(expectThrow(kMinimal + "node 2 5 5\nedge 1 2 0\n", "weight must be positive"));
        CHECK(expectThrow(kMinimal + "station x 9\n", "unknown node"));
        CHECK(expectThrow("ane-roadmap 1\nhall 0 0 1 1\n", "needs 'frame'"));
        CHECK(expectThrow(kMinimal + "node 3 1e999 0\n", "not a finite number"));
        CHECK(!expectThrow(kMinimal, "anything"));
    }});

    cases.push_back({"the engine's nominal route runs straight through the middle aisle", [] {
        const Roadmap& map = *warehouse();
        NavigationCore nav(&map.graph());
        CHECK(nav.startNavigation(0, 88));
        const std::vector<int> route = nav.getCurrentPath();
        CHECK(map.routeFollowsEdges(route));
        CHECK_NEAR(map.routeCost(route), 13.0, 1e-9);
        CHECK_NEAR(map.routeLength(route), 13.0, 1e-9);
        for (int id : route) {
            CHECK_NEAR(map.nodePosition(id).y, 0.0, 1e-9);
        }
        CHECK(map.routeCost({0, 88}) == std::numeric_limits<double>::infinity());
    }});

    cases.push_back({"the warehouse roadmap meets the footprint clearance contract", [] {
        const Roadmap& map = *warehouse();
        const Footprint footprint;
        CHECK_NEAR(footprint.circumscribedRadius(), 0.2724, 1e-4);
        const ClearanceRequirement need = clearanceRequirement(footprint, 0.20, 0.10);
        const ClearanceReport report = checkClearance(map, need);
        CHECK_NOTE(report.violations.empty(), report.violations.empty() ? "" : report.violations.front());
        CHECK(report.min_node_clearance >= need.node);
        CHECK(report.min_edge_clearance >= need.edge);
        // A node placed next to a rack violates it.
        const Roadmap tight = Roadmap::parse(
            "ane-roadmap 1\nframe 1 0 0 0\nhall -5 -5 5 5\nobstacle rack 0.2 -1 1 1 2 r\nnode 0 0 0\nnode 1 -1 0\nedge 0 1 1\n",
            "tight");
        CHECK(!checkClearance(tight, need).violations.empty());
    }});

    cases.push_back({"nearest-node association honours its radius", [] {
        const Roadmap& map = *warehouse();
        double d = 0.0;
        CHECK(map.nearestNode({-6.45, 0.08}, 0.2, &d) == 0);
        CHECK_NEAR(d, std::hypot(0.05, 0.08), 1e-12);
        CHECK(map.nearestNode({0.0, 1.5}, 0.2) == -1);  // inside a rack row
        const std::vector<int> inside = map.nodesInRect({-3.5, 2.5, 3.5, 3.5});
        CHECK(inside.size() == 15);
    }});

    cases.push_back({"node blocking covers every corridor the robot may sweep", [] {
        const Roadmap& map = *warehouse();
        const BlockageMapperConfig config;
        const double sweep = Footprint{}.circumscribedRadius() + 0.20;
        double worst = 0.0;
        for (const RoadmapEdge& e : map.edges()) {
            const Point2 a = map.nodePosition(e.from);
            const Point2 b = map.nodePosition(e.to);
            const double len = distance(a, b);
            const double nx = -(b.y - a.y) / len;
            const double ny = (b.x - a.x) / len;
            for (int k = 0; k <= 20; ++k) {
                const double t = k / 20.0;
                for (double side : {-1.0, 1.0}) {
                    const Point2 p{a.x + t * (b.x - a.x) + side * sweep * nx, a.y + t * (b.y - a.y) + side * sweep * ny};
                    worst = std::max(worst, std::min(distance(p, a), distance(p, b)));
                }
            }
        }
        CHECK_NOTE(worst <= config.blocking_radius, "worst " + std::to_string(worst));
    }});

    cases.push_back({"scans of the static warehouse block no node from any node", [] {
        const Roadmap& map = *warehouse();
        const std::vector<Rect> rects = staticRects(map);
        BlockageMapper mapper(map, {});
        const double inc = 2.0 * kPi / 360.0;
        for (int id : map.nodeIds()) {
            const Point2 p = map.nodePosition(id);
            for (double yaw : {0.0, 1.3}) {
                const Pose2 sensor{p.x, p.y, yaw};
                const auto ranges = ane::test::castScan(sensor, 360, -kPi, inc, 10.0, rects);
                for (int repeat = 0; repeat < 3; ++repeat) {
                    const ScanBlockages result = mapper.processScan(scanToMapPoints(sensor, -kPi, inc, ranges, 0.12, 10.0));
                    CHECK(result.newly_blocked.empty());
                    CHECK(result.unexplained.empty());
                }
            }
        }
        CHECK(mapper.blockedNodes().empty());
    }});

    cases.push_back({"a pallet in the middle aisle blocks the node in front of it after two scans", [] {
        const Roadmap& map = *warehouse();
        std::vector<Rect> rects = staticRects(map);
        rects.push_back({0.5, -0.6, 1.5, 0.6});
        BlockageMapper mapper(map, {});
        const double inc = 2.0 * kPi / 360.0;
        const Pose2 sensor{-2.0, 0.0, 0.0};
        const auto points = scanToMapPoints(sensor, -kPi, inc, ane::test::castScan(sensor, 360, -kPi, inc, 10.0, rects), 0.12, 10.0);
        const ScanBlockages first = mapper.processScan(points);
        CHECK(first.newly_blocked.empty());
        CHECK(!first.unexplained.empty());
        const ScanBlockages second = mapper.processScan(points);
        const int front = nodeAt(map, 0.5, 0.0);
        CHECK(std::find(second.newly_blocked.begin(), second.newly_blocked.end(), front) != second.newly_blocked.end());
        for (int id : second.newly_blocked) {
            CHECK(distance(map.nodePosition(id), {0.5, 0.0}) <= 0.55 + 1e-9);
        }
        CHECK(mapper.processScan(points).newly_blocked.empty());  // latched, reported once
        mapper.clear({front});
        CHECK(mapper.blockedNodes().count(front) == 0);
        // Two returns are below min_points.
        BlockageMapper sparse(map, {});
        const std::vector<Point2> two{{0.45, 0.0}, {0.5, 0.05}};
        sparse.processScan(two);
        CHECK(sparse.processScan(two).newly_blocked.empty());
    }});

    cases.push_back({"scan conversion skips invalid ranges and applies the sensor yaw", [] {
        const std::vector<float> ranges{1.0f, std::numeric_limits<float>::infinity(), 0.05f, 10.0f, 2.0f};
        const auto points = scanToMapPoints({1.0, 2.0, kPi / 2}, 0.0, kPi / 2, ranges, 0.12, 10.0);
        CHECK(points.size() == 2);
        CHECK_NEAR(points[0].x, 1.0, 1e-9);
        CHECK_NEAR(points[0].y, 3.0, 1e-9);  // beam 0 points along the sensor's +x, which is map +y
        CHECK_NEAR(points[1].x, 1.0, 1e-9);   // beam 4: yaw pi/2 + 2 pi = map +y
        CHECK_NEAR(points[1].y, 4.0, 1e-9);
    }});

    cases.push_back({"every controller output respects the velocity and acceleration limits", [] {
        ControllerConfig config;
        SegmentController controller(config);
        const ControllerLimits& l = config.limits;
        std::mt19937_64 rng(11);
        std::uniform_real_distribution<double> pos(-5.0, 5.0);
        std::uniform_real_distribution<double> ang(-kPi, kPi);
        std::uniform_real_distribution<double> step(0.005, 0.2);
        VelocityCommand previous{};
        for (int i = 0; i < 20000; ++i) {
            const double dt = step(rng);
            const Leg leg{{pos(rng), pos(rng)}, {pos(rng), pos(rng)}, i % 3 == 0 ? 0.0 : std::abs(pos(rng))};
            const VelocityCommand c = (i % 17 == 0) ? controller.brake(dt) : controller.track({pos(rng), pos(rng), ang(rng)}, leg, dt);
            CHECK(std::abs(c.linear) <= l.max_linear + 1e-12);
            CHECK(std::abs(c.angular) <= l.max_angular + 1e-12);
            CHECK(std::abs(c.linear - previous.linear) <= std::max(l.max_linear_accel, l.max_linear_decel) * dt + 1e-12);
            if (std::abs(c.linear) > std::abs(previous.linear) && c.linear * previous.linear >= 0.0) {
                CHECK(std::abs(c.linear - previous.linear) <= l.max_linear_accel * dt + 1e-12);
            }
            CHECK(std::abs(c.angular - previous.angular) <= l.max_angular_accel * dt + 1e-12);
            previous = c;
        }
    }});

    cases.push_back({"the controller turns in place, then drives and stops at the stop point", [] {
        SegmentController controller;
        ane::test::UnicyclePlant plant({0.0, 0.0, kPi}, {});
        const Leg leg{{0.0, 0.0}, {2.0, 0.0}, 0.0};
        bool rotatedInPlace = false;
        double maxSpeedWhileRotating = 0.0;
        for (int i = 0; i < 400; ++i) {
            const VelocityCommand c = controller.track(plant.pose(), leg, 0.05);
            if (controller.mode() == ControlMode::ROTATE) {
                rotatedInPlace = true;
                maxSpeedWhileRotating = std::max(maxSpeedWhileRotating, std::abs(c.linear));
            }
            for (int k = 0; k < 50; ++k) {
                plant.step(c, 0.001);
            }
        }
        CHECK(rotatedInPlace);
        CHECK(maxSpeedWhileRotating == 0.0);
        CHECK(std::abs(plant.pose().x - 2.0) <= 0.06);
        CHECK(std::abs(plant.pose().y) <= 0.02);
        CHECK(std::abs(plant.linear()) < 1e-6);
        CHECK(controller.mode() == ControlMode::DRIVE);
    }});

    cases.push_back({"a mission starts only from a pose associated with a node", [] {
        MissionExecutive exec(warehouse(), {});
        exec.updatePose(0.0, {-6.5, 0.5, 0.0});
        CHECK(!exec.start(0.0, 88).accepted);
        exec.updatePose(0.1, {-6.45, 0.05, 0.0});
        const StartResult started = exec.start(0.1, 88);
        CHECK(started.accepted);
        CHECK(started.start_node == 0);
        CHECK_NEAR(started.cost, 13.0, 1e-9);
        CHECK(exec.state() == MissionState::EXECUTING);
        CHECK(exec.engineState() == NavigationState::ACTIVE);
        CHECK(!exec.start(0.2, 88).accepted);  // one mission at a time
        const auto events = exec.takeEvents();
        CHECK(hasEvent(events, "START_REJECTED"));
        CHECK(hasEvent(events, "ROUTE_PLANNED"));
    }});

    cases.push_back({"stale feedback holds with zero velocity; recovery resumes; a long loss fails", [] {
        MissionExecutive exec(warehouse(), {});
        exec.updatePose(0.0, {-6.5, 0.0, 0.0});
        CHECK(exec.start(0.0, 88).accepted);
        double t = driveAlongMiddle(exec, -6.5, -6.2, 0.0);
        const VelocityCommand moving = exec.step(t + 0.05, 0.05);
        CHECK(moving.linear > 0.0);
        const VelocityCommand held = exec.step(t + 0.30, 0.05);  // pose now 0.30 s old
        CHECK(exec.state() == MissionState::HOLDING_FEEDBACK);
        CHECK(exec.engineState() == NavigationState::PAUSED);
        CHECK(held.linear == 0.0 && held.angular == 0.0);
        CHECK(exec.step(t + 2.0, 0.05).linear == 0.0);
        exec.updatePose(t + 2.1, {-6.2, 0.01, 0.0});
        exec.step(t + 2.1, 0.05);
        CHECK(exec.state() == MissionState::EXECUTING);
        CHECK(exec.engineState() == NavigationState::ACTIVE);
        exec.step(t + 2.5, 0.05);
        CHECK(exec.state() == MissionState::HOLDING_FEEDBACK);
        exec.step(t + 6.0, 0.05);
        CHECK(exec.state() == MissionState::HOLDING_FEEDBACK);
        exec.step(t + 6.6, 0.05);
        CHECK(exec.state() == MissionState::FAILED_LOCALIZATION);
        exec.updatePose(t + 7.0, {-6.2, 0.0, 0.0});
        const VelocityCommand after = exec.step(t + 7.0, 0.05);
        CHECK(after.linear == 0.0 && after.angular == 0.0);
        const auto events = exec.takeEvents();
        CHECK(hasEvent(events, "FEEDBACK_STALE"));
        CHECK(hasEvent(events, "FEEDBACK_RECOVERED"));
    }});

    cases.push_back({"blockages: the engine keeps, replaces or gives up the route", [] {
        const Roadmap& map = *warehouse();
        MissionExecutive exec(warehouse(), {});
        exec.updatePose(0.0, {-6.5, 0.0, 0.0});
        CHECK(exec.start(0.0, 88).accepted);
        exec.takeEvents();
        exec.reportBlockage(0.1, {nodeAt(map, 0.0, 3.0)}, "OPERATOR", "north aisle");
        CHECK(hasEvent(exec.takeEvents(), "ROUTE_KEPT"));
        const int middle = nodeAt(map, 0.5, 0.0);
        exec.reportBlockage(0.2, {middle}, "SENSOR", "pallet");
        CHECK(exec.replanCount() == 1);
        const std::vector<int> route = exec.route();
        CHECK(std::find(route.begin(), route.end(), middle) == route.end());
        CHECK(map.routeFollowsEdges(route));
        // The executive drives exactly what a fresh engine plans around the same blocked nodes.
        NavigationCore reference(&map.graph());
        reference.handleObstacles({nodeAt(map, 0.0, 3.0), middle});
        CHECK(reference.startNavigation(0, 88));
        CHECK(reference.getCurrentPath() == route);
        CHECK_NEAR(map.routeCost(route), 19.0, 1e-9);  // the south aisle; the north one costs 21
        for (int id : route) {
            CHECK(map.nodePosition(id).y <= 1e-9);
        }
        exec.reportBlockage(0.3, {nodeAt(map, 0.0, -3.0)}, "SENSOR", "south aisle");
        CHECK(exec.state() == MissionState::BLOCKED);
        CHECK(exec.engineState() == NavigationState::BLOCKED);
        CHECK(!exec.invalidatedRoute().empty());
        const VelocityCommand c = exec.step(0.35, 0.05);
        CHECK(c.linear == 0.0 && c.angular == 0.0);
        CHECK(!exec.retry(0.4));
        CHECK(exec.blockedNodes().size() == 3);
        CHECK(exec.blockageSources().at(middle) == "SENSOR");
    }});

    cases.push_back({"a replacement that turns back first returns to the current node", [] {
        const Roadmap& map = *warehouse();
        MissionExecutive exec(warehouse(), {});
        exec.updatePose(0.0, {-6.5, 0.0, 0.0});
        CHECK(exec.start(0.0, 88).accepted);
        double t = driveAlongMiddle(exec, -6.5, -2.75, 0.0);
        CHECK(exec.currentNode() == nodeAt(map, -3.0, 0.0));
        CHECK(exec.targetNode() == nodeAt(map, -2.5, 0.0));
        exec.takeEvents();
        exec.reportBlockage(t, {nodeAt(map, -2.5, 0.0)}, "SENSOR", "pallet ahead");
        const auto events = exec.takeEvents();
        CHECK(hasEvent(events, "ROUTE_REPLACED"));
        CHECK(hasEvent(events, "REJOIN"));
        CHECK(exec.returningToNode());
        CHECK(exec.targetNode() == nodeAt(map, -3.0, 0.0));
        t = driveAlongMiddle(exec, -2.75, -3.2, t);
        CHECK(!exec.returningToNode());
        CHECK(exec.currentNode() == nodeAt(map, -3.5, 0.0) || exec.currentNode() == nodeAt(map, -3.0, 0.0));
        CHECK(exec.state() == MissionState::EXECUTING);
    }});

    cases.push_back({"an operator stop latches zero velocity", [] {
        MissionExecutive exec(warehouse(), {});
        exec.updatePose(0.0, {-6.5, 0.0, 0.0});
        CHECK(exec.start(0.0, 88).accepted);
        driveAlongMiddle(exec, -6.5, -6.0, 0.0);
        exec.stop(2.0, "test");
        CHECK(exec.state() == MissionState::STOPPED);
        CHECK(exec.engineState() == NavigationState::PAUSED);
        exec.updatePose(2.05, {-6.0, 0.0, 0.0});
        const VelocityCommand c = exec.step(2.05, 0.05);
        CHECK(c.linear == 0.0 && c.angular == 0.0);
    }});

    return ane::test::runCases("test_adapter_core", cases);
}

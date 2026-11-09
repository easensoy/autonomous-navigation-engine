// Closed-loop test of the adapter core: every scenario in ros2/ane_sim/scenarios runs with each of its
// seeds against an ideal kinematic robot and a ray-cast lidar (kinematic_world.hpp), and must end in
// the outcome its acceptance criteria expect, without touching an obstacle.
//
// This checks the adapter's logic (association, leg tracking, node stepping, sensor blockages, engine
// replans, holds and failures) before anything runs in Gazebo. It has no physics, slip, noise or
// latency, so its numbers are test results, not simulation results.
#include "ane_core/blockage_mapper.hpp"
#include "ane_core/mission_executive.hpp"
#include "check.hpp"
#include "kinematic_world.hpp"
#include "scenario_file.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace ane;
using ane::test::Scenario;

namespace {

constexpr double kPi = 3.14159265358979323846;

struct RunResult {
    std::string outcome = "TIMEOUT";
    double final_error = -1.0;
    double min_clearance = 1e9;
    double max_cross_track = 0.0;
    int sensor_reports = 0;
    int operator_reports = 0;
    unsigned replans = 0;
    int holds = 0;
    int recoveries = 0;
    double hold_detect = -1.0;
    double blind_travel = 0.0;
    double failure_detect = -1.0;
    double mission_time = 0.0;
    double drift_after_stop = 0.0;
};

RunResult run(const Scenario& s, unsigned long long seed, const std::shared_ptr<const Roadmap>& map) {
    RunResult r;
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> ux(-s.jitter_x, s.jitter_x);
    std::uniform_real_distribution<double> uy(-s.jitter_y, s.jitter_y);
    std::uniform_real_distribution<double> uyaw(-s.jitter_yaw, s.jitter_yaw);
    const Point2 start = map->nodePosition(map->station(s.start));
    const double jx = ux(rng);
    const double jy = uy(rng);
    const double jyaw = uyaw(rng);
    ane::test::UnicyclePlant plant({start.x + jx, start.y + jy, jyaw}, {});
    MissionExecutive exec(map, {});
    BlockageMapper mapper(*map, {});
    std::vector<Rect> world;
    for (const StaticObstacle& o : map->obstacles()) {
        world.push_back(o.rect);
    }
    const Footprint footprint;
    const double inc = 2.0 * kPi / 360.0;
    const double dt = 0.001;
    const double missionStart = 1.0;
    bool started = false;
    std::vector<bool> fired(s.events.size(), false);
    double dropoutStart = -1.0, dropoutUntil = -1.0, stallStart = -1.0, stallUntil = -1.0;
    double terminalAt = -1.0;
    Point2 stopPoint{};
    bool stopped = false;
    Point2 lastPosition = plant.pose().position();
    double blindFrom = -1.0;
    VelocityCommand command{};

    for (long step = 0;; ++step) {
        const double t = step * dt;
        if (step % 20 == 0 && !(t >= dropoutStart && t < dropoutUntil)) {
            exec.updatePose(t, plant.pose());
        }
        if (step % 100 == 0 && exec.hasPose() && exec.feedbackAge(t) <= 0.1) {
            const auto ranges = ane::test::castScan(plant.pose(), 360, -kPi, inc, 10.0, world);
            const ScanBlockages b = mapper.processScan(scanToMapPoints(exec.pose(), -kPi, inc, ranges, 0.12, 10.0));
            if (!b.newly_blocked.empty()) {
                exec.reportBlockage(t, b.newly_blocked, "SENSOR", std::to_string(b.unexplained.size()) + " returns");
                ++r.sensor_reports;
            }
        }
        if (step % 50 == 0) {
            if (!started && t >= missionStart) {
                const StartResult result = exec.start(t, map->station(s.goal));
                CHECK_NOTE(result.accepted, s.name + ": " + result.message);
                started = true;
            }
            const double since = t - missionStart;
            for (std::size_t i = 0; started && i < s.events.size(); ++i) {
                const auto& e = s.events[i];
                const Pose2& p = plant.pose();
                const bool due = e.trigger == "at_start" || (e.trigger == "x_ge" && p.x >= e.trigger_value) ||
                                 (e.trigger == "x_le" && p.x <= e.trigger_value) ||
                                 (e.trigger == "y_ge" && p.y >= e.trigger_value) ||
                                 (e.trigger == "y_le" && p.y <= e.trigger_value) ||
                                 (e.trigger == "time_ge" && since >= e.trigger_value);
                if (fired[i] || !due) {
                    continue;
                }
                fired[i] = true;
                const auto& a = e.args;
                if (e.action == "spawn_obstacle") {
                    const double cx = std::stod(a[1]), cy = std::stod(a[2]), sx = std::stod(a[3]), sy = std::stod(a[4]);
                    world.push_back({cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2});
                } else if (e.action == "close_region") {
                    const Rect region{std::stod(a[1]), std::stod(a[2]), std::stod(a[3]), std::stod(a[4])};
                    exec.reportBlockage(t, map->nodesInRect(region), "OPERATOR", a[0]);
                    ++r.operator_reports;
                } else if (e.action == "odom_dropout") {
                    dropoutStart = t;
                    dropoutUntil = t + std::stod(a[0]);
                    blindFrom = t;
                } else if (e.action == "drive_stall") {
                    stallStart = t;
                    stallUntil = t + std::stod(a[0]);
                }
            }
            command = exec.step(t, 0.05);
            for (const MissionEvent& ev : exec.takeEvents()) {
                if (ev.type == "FEEDBACK_STALE") {
                    ++r.holds;
                    r.hold_detect = ev.time - dropoutStart;
                } else if (ev.type == "FEEDBACK_RECOVERED") {
                    ++r.recoveries;
                } else if (ev.type == "STATE" && ev.detail.find("-> FAILED_CONTROLLER") != std::string::npos && stallStart >= 0) {
                    r.failure_detect = ev.time - stallStart;
                }
            }
            if (exec.controller().mode() == ControlMode::DRIVE) {
                r.max_cross_track = std::max(r.max_cross_track, std::abs(exec.controller().crossTrack()));
            }
            if (terminalAt < 0.0 && isTerminal(exec.state())) {
                terminalAt = t;
            }
        }
        plant.step((t >= stallStart && t < stallUntil) ? VelocityCommand{} : command, dt);
        const Point2 now = plant.pose().position();
        if (blindFrom >= 0.0) {
            r.blind_travel += distance(now, lastPosition);
            if (std::abs(plant.linear()) < 0.01 && t > blindFrom + 0.05) {
                blindFrom = -1.0;
            }
        }
        lastPosition = now;
        if (step % 20 == 0) {
            r.min_clearance = std::min(r.min_clearance, ane::test::footprintClearance(plant.pose(), footprint.length, footprint.width, world));
        }
        if (terminalAt >= 0.0) {
            if (!stopped && std::abs(plant.linear()) < 0.01 && std::abs(plant.angular()) < 0.01) {
                stopped = true;
                stopPoint = now;
            }
            if (stopped) {
                r.drift_after_stop = std::max(r.drift_after_stop, distance(now, stopPoint));
            }
            if (t > terminalAt + s.observe) {
                break;
            }
        }
        if (t > missionStart + s.timeout) {
            break;
        }
    }
    if (terminalAt >= 0.0) {
        r.outcome = missionStateName(exec.state());
        r.mission_time = terminalAt - missionStart;
    }
    r.replans = exec.replanCount();
    r.final_error = distance(plant.pose().position(), map->nodePosition(map->station(s.goal)));
    return r;
}

}  // namespace

int main() {
    const auto map = std::make_shared<const Roadmap>(Roadmap::load(ANE_ROADMAP_FILE));
    const std::vector<std::string> files = {"A_nominal", "B_aisle_blockage", "C_disconnection",
                                            "D_feedback_dropout", "E_feedback_loss", "F_drive_stall"};
    std::vector<ane::test::Case> cases;
    for (const std::string& file : files) {
        const Scenario s = ane::test::loadScenario(std::string(ANE_SCENARIO_DIR) + "/" + file + ".txt");
        cases.push_back({s.name, [s, map] {
            const std::string expected = s.expected("outcome");
            CHECK(!expected.empty());
            for (int k = 0; k < s.trials; ++k) {
                const RunResult r = run(s, s.seed + static_cast<unsigned long long>(k), map);
                std::cout << "  " << s.name << " seed " << (s.seed + k) << ": " << r.outcome << std::fixed
                          << std::setprecision(3) << " | t " << r.mission_time << " s | final error " << r.final_error
                          << " m | min clearance " << r.min_clearance << " m | max cross-track " << r.max_cross_track
                          << " m | sensor reports " << r.sensor_reports << " | replans " << r.replans << " | holds "
                          << r.holds << std::endl;
                CHECK_NOTE(r.outcome == expected, s.name + " seed " + std::to_string(s.seed + k) + ": " + r.outcome);
                CHECK_NOTE(r.min_clearance > 0.0, s.name + ": the footprint touched an obstacle");
                CHECK(r.max_cross_track <= 0.20);
                if (expected == "COMPLETED") {
                    CHECK(r.final_error <= 0.10);
                }
                if (s.name == "A_nominal") {
                    CHECK(r.replans == 0 && r.sensor_reports == 0);
                }
                if (s.name == "B_aisle_blockage") {
                    CHECK(r.sensor_reports >= 1 && r.replans >= 1 && r.operator_reports == 0);
                }
                if (s.name == "C_disconnection") {
                    CHECK(r.operator_reports == 2 && r.sensor_reports >= 1);
                    CHECK(r.drift_after_stop <= 0.05);
                }
                if (s.name == "D_feedback_dropout") {
                    CHECK(r.holds == 1 && r.recoveries == 1);
                    CHECK(r.hold_detect <= 0.35);
                    CHECK(r.blind_travel <= 0.35);
                }
                if (s.name == "E_feedback_loss") {
                    CHECK(r.holds == 1 && r.hold_detect <= 0.35 && r.blind_travel <= 0.35);
                    CHECK(r.drift_after_stop <= 0.05);
                }
                if (s.name == "F_drive_stall") {
                    CHECK(r.failure_detect > 0.0 && r.failure_detect <= 5.0);
                    CHECK(r.drift_after_stop <= 0.05);
                }
            }
        }});
    }
    return ane::test::runCases("test_closed_loop", cases);
}

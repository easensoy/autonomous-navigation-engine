#pragma once
// Executes one mission on the roadmap with this repository's NavigationCore.
//
// The engine plans and replans; the executive turns its node route into motion and feeds back what
// happens:
//   start          associates the robot pose with the nearest node (within association_radius) and calls
//                  NavigationCore::startNavigation(start, goal) (A* through the DynamicReplanner).
//   step           drives the current leg with the SegmentController and calls
//                  NavigationCore::executeNavigationStep() each time the robot reaches the next node.
//   reportBlockage passes newly blocked nodes to NavigationCore::handleObstacles(). The engine keeps the
//                  route if it is clear, replaces the route ahead from the current node otherwise, or
//                  enters BLOCKED when no route is left.
// The executive never edits a route: every node sequence it drives is NavigationCore::getCurrentPath().
// After a replacement whose next node differs from the one the robot was heading to, a robot more than
// rejoin_radius from the current node first drives back to it along the edge it was on (it never
// leaves the roadmap to cut across).
//
// Failure behaviour (every terminal state commands zero velocity on every later step):
//   BLOCKED              NavigationCore entered BLOCKED: no route around the reported blockages.
//   HOLDING_FEEDBACK     the newest pose is older than feedback_timeout: zero velocity, engine paused.
//                        Fresh feedback within recovery_window resumes, provided the pose is still within
//                        max_cross_track of the leg; otherwise FAILED_LOCALIZATION.
//   FAILED_CONTROLLER    no progress for progress_timeout while motion is commanded, or a cross-track error
//                        above max_cross_track while driving.
//   STOPPED              operator stop; latched until a new mission.
#include "ane_core/roadmap.hpp"
#include "ane_core/segment_controller.hpp"
#include "autonomous_navigator/NavigationCore.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ane {

enum class MissionState {
    IDLE,
    EXECUTING,
    HOLDING_FEEDBACK,
    COMPLETED,
    BLOCKED,
    FAILED_CONTROLLER,
    FAILED_LOCALIZATION,
    STOPPED
};
const char* missionStateName(MissionState state);
bool isTerminal(MissionState state);  // COMPLETED, BLOCKED, FAILED_CONTROLLER, FAILED_LOCALIZATION, STOPPED

struct ExecutiveConfig {
    ControllerConfig controller;
    double association_radius = 0.20;   // m: the start pose must be this close to a node
    double goal_tolerance = 0.10;       // m: final position error accepted at the goal
    double stop_arrival_radius = 0.06;  // m: a turn node is reached this close (or once passed)
    double max_cross_track = 0.20;      // m
    double progress_timeout = 4.0;      // s
    double progress_distance = 0.05;    // m, or rad while rotating in place
    double feedback_timeout = 0.25;     // s
    double recovery_window = 4.0;       // s
    double rejoin_radius = 0.15;        // m
    double turn_threshold = 0.1745;     // rad (10 deg): a larger change of direction makes a node a stop point
};

struct MissionEvent {
    double time = 0.0;      // node clock: simulation time when run in Gazebo, s
    long long wall_ns = 0;  // system clock, ns since the epoch
    std::string type;       // see ros2/README.md, "Events"
    std::string source;     // SENSOR, OPERATOR, SCRIPTED, ENGINE or EXECUTIVE
    std::vector<int> nodes;
    double value = 0.0;     // type-specific: engine call time in ms (wall), distance in m, duration in s
    std::string detail;
};

struct StartResult {
    bool accepted = false;
    std::string message;
    int start_node = -1;
    int goal_node = -1;
    std::vector<int> route;
    double cost = 0.0;
};

class MissionExecutive {
public:
    MissionExecutive(std::shared_ptr<const Roadmap> roadmap, ExecutiveConfig config);

    StartResult start(double now, int goalNode);
    void updatePose(double stamp, const Pose2& pose);
    void reportBlockage(double now, const std::vector<int>& nodes, const std::string& source, const std::string& detail);
    void stop(double now, const std::string& reason);
    bool retry(double now);
    VelocityCommand step(double now, double dt);

    MissionState state() const { return state_; }
    NavigationState engineState() const { return nav_.getNavigationState(); }
    std::vector<int> route() const { return nav_.getCurrentPath(); }
    std::vector<int> invalidatedRoute() const { return nav_.getInvalidatedRoute(); }
    std::vector<int> blockedNodes() const;
    const std::map<int, std::string>& blockageSources() const { return blocked_source_; }
    int startNode() const { return start_node_; }
    int goalNode() const { return goal_node_; }
    int currentNode() const { return nav_.getCurrentPosition(); }
    int targetNode() const;  // node the robot is driving to, -1 when not executing
    bool returningToNode() const { return returning_; }
    std::size_t routeIndex() const { return index_; }
    unsigned routeVersion() const { return route_version_; }
    unsigned replanCount() const { return replans_; }
    double remainingCost() const { return nav_.getRemainingDistance(); }
    double feedbackAge(double now) const;
    bool hasPose() const { return have_pose_; }
    const Pose2& pose() const { return pose_; }
    double poseStamp() const { return pose_stamp_; }
    const Leg& leg() const { return leg_; }
    const SegmentController& controller() const { return controller_; }
    const Roadmap& roadmap() const { return *roadmap_; }
    const ExecutiveConfig& config() const { return config_; }
    std::vector<MissionEvent> takeEvents();

private:
    void raise(double now, const std::string& type, const std::string& source, std::vector<int> nodes = {},
               double value = 0.0, const std::string& detail = {});
    void enter(double now, MissionState next, const std::string& reason);
    Point2 node(int id) const { return roadmap_->nodePosition(id); }
    double directionChange(const Point2& a, const Point2& b, const Point2& c) const;
    double stopDistanceAfter(std::size_t index) const;
    void refreshLeg();
    bool atLegEnd() const;
    void advance(double now);
    double progressMetric() const;
    void resetProgress(double now);
    void checkProgress(double now, const VelocityCommand& command);
    void syncRoute();

    std::shared_ptr<const Roadmap> roadmap_;
    ExecutiveConfig config_;
    NavigationCore nav_;
    SegmentController controller_;
    MissionState state_ = MissionState::IDLE;

    bool have_pose_ = false;
    double pose_stamp_ = 0.0;
    Pose2 pose_{};

    std::vector<int> route_;
    std::size_t index_ = 0;
    int start_node_ = -1;
    int goal_node_ = -1;
    bool returning_ = false;
    Point2 return_from_{};
    bool finishing_ = false;
    Leg leg_{};
    int leg_target_ = -1;
    double hold_start_ = 0.0;

    std::map<int, std::string> blocked_source_;
    unsigned route_version_ = 0;
    unsigned replans_ = 0;

    ControlMode progress_mode_ = ControlMode::IDLE;
    double progress_best_ = 0.0;
    double progress_time_ = 0.0;

    std::vector<MissionEvent> events_;
};

}  // namespace ane

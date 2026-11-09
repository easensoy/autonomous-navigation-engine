#include "ane_core/mission_executive.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace ane {

namespace {

double elapsedMs(std::chrono::steady_clock::time_point since) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - since).count();
}

long long wallNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::string fixed(double value, int decimals = 3) {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(decimals);
    out << value;
    return out.str();
}

bool moving(const VelocityCommand& c) {
    return std::abs(c.linear) > 1e-3 || std::abs(c.angular) > 1e-3;
}

}  // namespace

const char* missionStateName(MissionState state) {
    switch (state) {
        case MissionState::IDLE: return "IDLE";
        case MissionState::EXECUTING: return "EXECUTING";
        case MissionState::HOLDING_FEEDBACK: return "HOLDING_FEEDBACK";
        case MissionState::COMPLETED: return "COMPLETED";
        case MissionState::BLOCKED: return "BLOCKED";
        case MissionState::FAILED_CONTROLLER: return "FAILED_CONTROLLER";
        case MissionState::FAILED_LOCALIZATION: return "FAILED_LOCALIZATION";
        case MissionState::STOPPED: return "STOPPED";
    }
    return "UNKNOWN";
}

bool isTerminal(MissionState state) {
    return state == MissionState::COMPLETED || state == MissionState::BLOCKED ||
           state == MissionState::FAILED_CONTROLLER || state == MissionState::FAILED_LOCALIZATION ||
           state == MissionState::STOPPED;
}

MissionExecutive::MissionExecutive(std::shared_ptr<const Roadmap> roadmap, ExecutiveConfig config)
    : roadmap_(std::move(roadmap)),
      config_(config),
      nav_(roadmap_ ? &roadmap_->graph() : nullptr),
      controller_(config.controller) {
    if (!roadmap_) {
        throw std::invalid_argument("mission executive needs a roadmap");
    }
    if (!(config_.association_radius > 0 && config_.goal_tolerance > 0 && config_.max_cross_track > 0 &&
          config_.progress_timeout > 0 && config_.feedback_timeout > 0 && config_.recovery_window > 0 &&
          config_.stop_arrival_radius > 0 && config_.rejoin_radius >= 0 && config_.turn_threshold > 0)) {
        throw std::invalid_argument("mission executive: tolerances and timeouts must be positive");
    }
}

void MissionExecutive::raise(double now, const std::string& type, const std::string& source, std::vector<int> nodes,
                             double value, const std::string& detail) {
    events_.push_back({now, wallNs(), type, source, std::move(nodes), value, detail});
}

void MissionExecutive::enter(double now, MissionState next, const std::string& reason) {
    if (next == state_) {
        return;
    }
    const std::string detail = std::string(missionStateName(state_)) + " -> " + missionStateName(next) + ": " + reason;
    state_ = next;
    if (next == MissionState::FAILED_CONTROLLER || next == MissionState::FAILED_LOCALIZATION ||
        next == MissionState::STOPPED) {
        nav_.emergencyStop();  // ACTIVE -> PAUSED; the engine keeps the route
    }
    if (isTerminal(next)) {
        controller_.reset();
    }
    raise(now, "STATE", "EXECUTIVE", {}, 0.0, detail);
}

std::vector<int> MissionExecutive::blockedNodes() const {
    std::vector<int> nodes;
    for (const auto& entry : blocked_source_) {
        nodes.push_back(entry.first);
    }
    return nodes;
}

int MissionExecutive::targetNode() const {
    if (state_ != MissionState::EXECUTING && state_ != MissionState::HOLDING_FEEDBACK) {
        return -1;
    }
    return leg_target_;
}

double MissionExecutive::feedbackAge(double now) const {
    return have_pose_ ? now - pose_stamp_ : std::numeric_limits<double>::infinity();
}

std::vector<MissionEvent> MissionExecutive::takeEvents() {
    std::vector<MissionEvent> out;
    out.swap(events_);
    return out;
}

void MissionExecutive::updatePose(double stamp, const Pose2& pose) {
    if (have_pose_ && stamp < pose_stamp_) {
        return;  // out of order
    }
    have_pose_ = true;
    pose_stamp_ = stamp;
    pose_ = pose;
}

double MissionExecutive::directionChange(const Point2& a, const Point2& b, const Point2& c) const {
    const double first = std::atan2(b.y - a.y, b.x - a.x);
    const double second = std::atan2(c.y - b.y, c.x - b.x);
    return std::abs(wrapAngle(second - first));
}

double MissionExecutive::stopDistanceAfter(std::size_t index) const {
    // Distance from route_[index] to the next stop point: the goal or a node where the route turns.
    double total = 0.0;
    for (std::size_t k = index; k + 1 < route_.size(); ++k) {
        if (k == 0 || directionChange(node(route_[k - 1]), node(route_[k]), node(route_[k + 1])) > config_.turn_threshold) {
            break;
        }
        total += distance(node(route_[k]), node(route_[k + 1]));
    }
    return total;
}

void MissionExecutive::refreshLeg() {
    if (returning_) {
        const Point2 to = node(route_[index_]);
        double stop = 0.0;
        if (index_ + 1 < route_.size() &&
            directionChange(return_from_, to, node(route_[index_ + 1])) <= config_.turn_threshold) {
            stop = distance(to, node(route_[index_ + 1])) + stopDistanceAfter(index_ + 1);
        }
        leg_ = {return_from_, to, stop};
        leg_target_ = route_[index_];
        return;
    }
    if (index_ + 1 >= route_.size()) {
        const Point2 here = route_.empty() ? pose_.position() : node(route_[index_]);
        leg_ = {here, here, 0.0};
        leg_target_ = route_.empty() ? -1 : route_[index_];
        return;
    }
    leg_ = {node(route_[index_]), node(route_[index_ + 1]), stopDistanceAfter(index_ + 1)};
    leg_target_ = route_[index_ + 1];
}

bool MissionExecutive::atLegEnd() const {
    const LineCoordinates lc = lineCoordinates(pose_.position(), leg_.start, leg_.end);
    const double d = distance(pose_.position(), leg_.end);
    const bool goalLeg = !returning_ && index_ + 2 == route_.size();
    if (goalLeg) {
        return d <= config_.goal_tolerance;
    }
    if (leg_.stop_distance == 0.0) {
        return d <= config_.stop_arrival_radius ||
               (lc.along >= lc.length - config_.controller.stop_tolerance - 1e-3 &&
                std::abs(lc.cross) <= config_.max_cross_track);
    }
    return lc.along >= lc.length;
}

void MissionExecutive::syncRoute() {
    route_ = nav_.getCurrentPath();
    if (index_ >= route_.size() || route_[index_] != nav_.getCurrentPosition()) {
        throw std::logic_error("mission executive lost track of NavigationCore's position");
    }
}

void MissionExecutive::advance(double now) {
    for (int guard = 0; guard < 8 && !finishing_ && atLegEnd(); ++guard) {
        if (returning_) {
            returning_ = false;
            raise(now, "RETURNED_TO_NODE", "EXECUTIVE", {route_[index_]});
            refreshLeg();
            resetProgress(now);
            continue;
        }
        const LineCoordinates lc = lineCoordinates(pose_.position(), leg_.start, leg_.end);
        if (!nav_.executeNavigationStep()) {
            throw std::logic_error("NavigationCore refused a step while the mission was executing");
        }
        ++index_;
        syncRoute();
        raise(now, "NODE_REACHED", "EXECUTIVE", {route_[index_]}, lc.cross);
        if (nav_.getNavigationState() == NavigationState::COMPLETED) {
            finishing_ = true;
            break;
        }
        refreshLeg();
        resetProgress(now);
    }
}

double MissionExecutive::progressMetric() const {
    if (controller_.mode() == ControlMode::ROTATE) {
        return std::abs(controller_.headingError());
    }
    return distance(pose_.position(), leg_.end);
}

void MissionExecutive::resetProgress(double now) {
    progress_mode_ = controller_.mode();
    progress_best_ = progressMetric();
    progress_time_ = now;
}

void MissionExecutive::checkProgress(double now, const VelocityCommand& command) {
    if (controller_.mode() != progress_mode_) {
        resetProgress(now);
        return;
    }
    const bool commanding = std::abs(command.linear) >= 0.02 || std::abs(command.angular) >= 0.05;
    const double metric = progressMetric();
    if (!commanding || metric < progress_best_ - config_.progress_distance) {
        progress_best_ = metric;
        progress_time_ = now;
        return;
    }
    if (now - progress_time_ > config_.progress_timeout) {
        enter(now, MissionState::FAILED_CONTROLLER,
              "no progress for " + fixed(now - progress_time_, 2) + " s while commanding v=" + fixed(command.linear) +
                  " m/s w=" + fixed(command.angular) + " rad/s");
    }
}

StartResult MissionExecutive::start(double now, int goalNode) {
    StartResult result;
    result.goal_node = goalNode;
    if (state_ == MissionState::EXECUTING || state_ == MissionState::HOLDING_FEEDBACK) {
        result.message = "a mission is in progress";
        return result;
    }
    if (feedbackAge(now) > config_.feedback_timeout) {
        result.message = "no fresh pose feedback";
        raise(now, "START_REJECTED", "EXECUTIVE", {}, 0.0, result.message);
        return result;
    }
    if (!roadmap_->hasNode(goalNode)) {
        result.message = "goal node " + std::to_string(goalNode) + " is not on the roadmap";
        raise(now, "START_REJECTED", "EXECUTIVE", {}, 0.0, result.message);
        return result;
    }
    double offset = 0.0;
    const int startNode = roadmap_->nearestNode(pose_.position(), config_.association_radius, &offset);
    if (startNode < 0) {
        result.message = "robot is " + fixed(offset) + " m from the nearest node (association radius " +
                         fixed(config_.association_radius) + " m)";
        raise(now, "START_REJECTED", "EXECUTIVE", {}, offset, result.message);
        return result;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const bool planned = nav_.startNavigation(startNode, goalNode);
    const double ms = elapsedMs(t0);
    if (!planned) {
        result.message = "the engine found no route from node " + std::to_string(startNode) + " to node " +
                         std::to_string(goalNode);
        raise(now, "START_REJECTED", "ENGINE", {startNode, goalNode}, ms, result.message);
        return result;
    }
    route_ = nav_.getCurrentPath();
    index_ = 0;
    start_node_ = startNode;
    goal_node_ = goalNode;
    returning_ = false;
    finishing_ = false;
    ++route_version_;
    controller_.reset();
    state_ = MissionState::EXECUTING;
    refreshLeg();
    resetProgress(now);
    raise(now, "MISSION_STARTED", "EXECUTIVE", {startNode, goalNode}, offset,
          "start node " + std::to_string(startNode) + " at " + fixed(offset) + " m from the robot");
    raise(now, "ROUTE_PLANNED", "ENGINE", route_, ms,
          "cost " + fixed(roadmap_->routeCost(route_)) + " length " + fixed(roadmap_->routeLength(route_)) + " m");
    if (nav_.getNavigationState() == NavigationState::COMPLETED) {
        finishing_ = true;  // start == goal
    }
    result.accepted = true;
    result.start_node = startNode;
    result.route = route_;
    result.cost = roadmap_->routeCost(route_);
    result.message = "route of " + std::to_string(route_.size()) + " nodes";
    return result;
}

void MissionExecutive::reportBlockage(double now, const std::vector<int>& nodes, const std::string& source,
                                      const std::string& detail) {
    std::vector<int> fresh;
    for (int id : nodes) {
        if (roadmap_->hasNode(id) && blocked_source_.emplace(id, source).second) {
            fresh.push_back(id);
        }
    }
    if (fresh.empty()) {
        return;
    }
    std::sort(fresh.begin(), fresh.end());
    raise(now, "BLOCKAGE_REPORTED", source, fresh, 0.0, detail);
    const bool active = state_ == MissionState::EXECUTING || state_ == MissionState::HOLDING_FEEDBACK;
    if (!active || finishing_) {
        nav_.handleObstacles(fresh);  // recorded for later plans; nothing to repair
        return;
    }
    const std::vector<int> before = nav_.getCurrentPath();
    const int previousTarget = leg_target_;
    const auto t0 = std::chrono::steady_clock::now();
    nav_.handleObstacles(fresh);
    const double ms = elapsedMs(t0);
    const std::vector<int> after = nav_.getCurrentPath();
    if (nav_.getNavigationState() == NavigationState::BLOCKED) {
        route_ = after;
        raise(now, "ROUTE_BLOCKED", "ENGINE", nav_.getInvalidatedRoute(), ms,
              "no route from node " + std::to_string(nav_.getCurrentPosition()) + " to goal " + std::to_string(goal_node_));
        enter(now, MissionState::BLOCKED, "the engine found no route around the blockages");
        return;
    }
    if (after == before) {
        raise(now, "ROUTE_KEPT", "ENGINE", {}, ms, "route ahead is clear");
        return;
    }
    ++replans_;
    ++route_version_;
    syncRoute();
    const std::vector<int> ahead(route_.begin() + static_cast<std::ptrdiff_t>(index_), route_.end());
    raise(now, "ROUTE_REPLACED", "ENGINE", ahead, ms,
          "cost ahead " + fixed(roadmap_->routeCost(ahead)) + " length ahead " + fixed(roadmap_->routeLength(ahead)) + " m");
    if (!returning_) {
        const int newTarget = index_ + 1 < route_.size() ? route_[index_ + 1] : -1;
        const double away = distance(pose_.position(), node(route_[index_]));
        if (newTarget != previousTarget && previousTarget >= 0 && away > config_.rejoin_radius) {
            returning_ = true;
            return_from_ = node(previousTarget);
            raise(now, "REJOIN", "EXECUTIVE", {route_[index_]}, away,
                  "driving back to node " + std::to_string(route_[index_]) + " along the edge from node " +
                      std::to_string(previousTarget));
        }
    }
    refreshLeg();
    resetProgress(now);
}

void MissionExecutive::stop(double now, const std::string& reason) {
    raise(now, "STOP_REQUESTED", "OPERATOR", {}, 0.0, reason);
    enter(now, MissionState::STOPPED, reason);
}

bool MissionExecutive::retry(double now) {
    if (state_ != MissionState::BLOCKED) {
        return false;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = nav_.retryNavigation();
    const double ms = elapsedMs(t0);
    if (!ok) {
        raise(now, "RETRY_FAILED", "ENGINE", {}, ms, "still no route");
        return false;
    }
    ++route_version_;
    syncRoute();
    returning_ = false;
    finishing_ = nav_.getNavigationState() == NavigationState::COMPLETED;
    state_ = MissionState::EXECUTING;
    raise(now, "ROUTE_RESTORED", "ENGINE", route_, ms);
    raise(now, "STATE", "EXECUTIVE", {}, 0.0, "BLOCKED -> EXECUTING: retry found a route");
    refreshLeg();
    resetProgress(now);
    return true;
}

VelocityCommand MissionExecutive::step(double now, double dt) {
    if (state_ != MissionState::EXECUTING && state_ != MissionState::HOLDING_FEEDBACK) {
        controller_.reset();
        return {};
    }
    const bool stale = feedbackAge(now) > config_.feedback_timeout;
    if (state_ == MissionState::EXECUTING && stale) {
        hold_start_ = now;
        nav_.pauseNavigation();
        raise(now, "FEEDBACK_STALE", "EXECUTIVE", {}, feedbackAge(now), "pose feedback older than " + fixed(config_.feedback_timeout, 2) + " s");
        enter(now, MissionState::HOLDING_FEEDBACK, "pose feedback stale; zero velocity");
        controller_.reset();
        return {};
    }
    if (state_ == MissionState::HOLDING_FEEDBACK) {
        if (stale) {
            if (now - hold_start_ > config_.recovery_window) {
                enter(now, MissionState::FAILED_LOCALIZATION,
                      "no pose feedback for " + fixed(now - hold_start_, 2) + " s (recovery window " +
                          fixed(config_.recovery_window, 1) + " s)");
            }
            return {};
        }
        const LineCoordinates lc = lineCoordinates(pose_.position(), leg_.start, leg_.end);
        if (std::abs(lc.cross) > config_.max_cross_track) {
            enter(now, MissionState::FAILED_LOCALIZATION,
                  "pose after recovery is " + fixed(std::abs(lc.cross)) + " m from the route");
            return {};
        }
        nav_.resumeNavigation();
        raise(now, "FEEDBACK_RECOVERED", "EXECUTIVE", {}, now - hold_start_);
        enter(now, MissionState::EXECUTING, "pose feedback recovered");
        resetProgress(now);
    }

    advance(now);
    if (finishing_) {
        const VelocityCommand out = controller_.brake(dt);
        if (!moving(out)) {
            const double error = distance(pose_.position(), node(goal_node_));
            raise(now, "MISSION_COMPLETED", "EXECUTIVE", {goal_node_}, error,
                  "final position error " + fixed(error) + " m");
            enter(now, MissionState::COMPLETED, "goal reached");
        }
        return out;
    }
    const VelocityCommand out = controller_.track(pose_, leg_, dt);
    if (controller_.mode() == ControlMode::DRIVE && std::abs(controller_.crossTrack()) > config_.max_cross_track) {
        enter(now, MissionState::FAILED_CONTROLLER,
              "cross-track error " + fixed(std::abs(controller_.crossTrack())) + " m above " +
                  fixed(config_.max_cross_track, 2) + " m");
        return {};
    }
    if (controller_.mode() == ControlMode::DRIVE && controller_.distanceToStop() <= config_.controller.stop_tolerance &&
        !moving(out) && !atLegEnd()) {
        // Stopped where the route says the leg ends, but too far from the node to count as reached.
        enter(now, MissionState::FAILED_CONTROLLER,
              "stopped " + fixed(distance(pose_.position(), leg_.end)) + " m from node " + std::to_string(leg_target_));
        return {};
    }
    checkProgress(now, out);
    if (state_ != MissionState::EXECUTING) {
        return {};
    }
    return out;
}

}  // namespace ane

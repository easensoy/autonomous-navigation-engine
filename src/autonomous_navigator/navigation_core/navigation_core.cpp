#include "autonomous_navigator/NavigationCore.hpp"
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

// NavigationCore owns the route being followed and the navigation state (see NavigationCore.hpp).
// Routes come from the DynamicReplanner, which plans with A* around every obstacle reported so far.
// A navigation step moves the current position one node along the route. Progress is kept as an
// index into the route, because a route through several waypoints can pass the same node twice.

namespace {
const char* modeName(NavigationMode mode) {
    switch (mode) {
        case NavigationMode::GLOBAL_ONLY: return "GLOBAL_ONLY";
        case NavigationMode::LOCAL_ONLY: return "LOCAL_ONLY";
        case NavigationMode::HYBRID: return "HYBRID";
        case NavigationMode::DYNAMIC: return "DYNAMIC";
    }
    return "UNKNOWN";
}
}  // namespace

const char* navigationStateName(NavigationState state) {
    switch (state) {
        case NavigationState::IDLE: return "IDLE";
        case NavigationState::ACTIVE: return "ACTIVE";
        case NavigationState::PAUSED: return "PAUSED";
        case NavigationState::COMPLETED: return "COMPLETED";
        case NavigationState::BLOCKED: return "BLOCKED";
    }
    return "UNKNOWN";
}

NavigationCore::NavigationCore(const Graph* graph)
    : environment(graph),
      currentMode(NavigationMode::GLOBAL_ONLY),
      state(NavigationState::IDLE),
      currentPosition(-1),
      goalNode(-1),
      routeIndex(0) {
    if (environment) {
        globalPlanner = std::make_unique<GlobalPathPlanner>(environment);
        dynamicReplanner = std::make_unique<DynamicReplanner>(environment);
    }
}

std::vector<int> NavigationCore::planRoute(int startId, int goalId) {
    if (!environment || !environment->hasNode(startId) || !environment->hasNode(goalId)) {
        return {};
    }
    if (startId == goalId) {
        return {startId};
    }
    // The replanner plans around every obstacle reported so far (plain A* when there are none).
    dynamicReplanner->setInitialPath({});
    return dynamicReplanner->updateAndReplan(startId, goalId, {});
}

bool NavigationCore::activatePath(std::vector<int> path) {
    if (path.empty()) {
        return false;  // a failed start leaves the current navigation as it was
    }
    activePath = std::move(path);
    invalidatedRoute.clear();
    routeIndex = 0;
    currentPosition = activePath.front();
    goalNode = activePath.back();
    state = activePath.size() > 1 ? NavigationState::ACTIVE : NavigationState::COMPLETED;
    dynamicReplanner->setInitialPath(activePath);
    return true;
}

bool NavigationCore::startNavigation(int startId, int goalId) {
    if (!environment) {
        std::cout << "[NAVIGATION] Cannot start: no environment" << std::endl;
        return false;
    }
    if (!environment->hasNode(startId) || !environment->hasNode(goalId)) {
        std::cout << "[NAVIGATION] Cannot start: node " << (environment->hasNode(startId) ? goalId : startId)
                  << " does not exist" << std::endl;
        return false;
    }
    return activatePath(planRoute(startId, goalId));
}

bool NavigationCore::startNavigationWithWaypoints(int startId, const std::vector<int>& waypoints) {
    if (!environment || waypoints.empty() || !environment->hasNode(startId)) {
        return false;
    }
    std::vector<int> route{startId};
    for (int waypoint : waypoints) {
        if (!environment->hasNode(waypoint)) {
            return false;
        }
        if (waypoint == route.back()) {
            continue;
        }
        std::vector<int> leg = planRoute(route.back(), waypoint);
        if (leg.empty()) {
            return false;
        }
        route.insert(route.end(), leg.begin() + 1, leg.end());
    }
    return activatePath(std::move(route));
}

void NavigationCore::stopNavigation() {
    state = NavigationState::IDLE;
    activePath.clear();
    invalidatedRoute.clear();
    routeIndex = 0;
    currentPosition = -1;
    goalNode = -1;
}

void NavigationCore::pauseNavigation() {
    if (state == NavigationState::ACTIVE) {
        state = NavigationState::PAUSED;
    }
}

void NavigationCore::resumeNavigation() {
    if (state == NavigationState::PAUSED) {
        state = NavigationState::ACTIVE;
    }
}

NavigationState NavigationCore::getNavigationState() const {
    return state;
}

std::vector<int> NavigationCore::getCurrentPath() const {
    return activePath;
}

std::vector<int> NavigationCore::getInvalidatedRoute() const {
    return invalidatedRoute;
}

int NavigationCore::getCurrentPosition() const {
    return currentPosition;
}

int NavigationCore::getGoal() const {
    return goalNode;
}

bool NavigationCore::isNavigationComplete() const {
    return state == NavigationState::COMPLETED;
}

double NavigationCore::getRemainingDistance() const {
    if (state == NavigationState::BLOCKED) {
        return std::numeric_limits<double>::infinity();
    }
    double remaining = 0.0;
    for (size_t i = routeIndex; i + 1 < activePath.size(); ++i) {
        remaining += environment->getEdgeWeight(activePath[i], activePath[i + 1]);
    }
    return remaining;
}

void NavigationCore::setNavigationMode(NavigationMode mode) {
    currentMode = mode;
}

void NavigationCore::updateEnvironment(const Graph* newEnvironment) {
    stopNavigation();
    environment = newEnvironment;
    globalPlanner.reset();
    dynamicReplanner.reset();
    if (environment) {
        globalPlanner = std::make_unique<GlobalPathPlanner>(environment);
        dynamicReplanner = std::make_unique<DynamicReplanner>(environment);
    }
}

void NavigationCore::handleObstacles(const std::vector<int>& obstacles) {
    if (!dynamicReplanner || obstacles.empty()) {
        return;
    }
    if (state == NavigationState::IDLE || state == NavigationState::BLOCKED) {
        // Nothing executable to repair: record the nodes so the next plan or retry avoids them.
        for (int obstacle : obstacles) {
            dynamicReplanner->addPersistentObstacle(obstacle);
        }
        return;
    }
    // Hand the replanner only the part still ahead. It keeps that part if it is still clear, or plans
    // a new one from the current position; either way the result starts at the current position.
    dynamicReplanner->setInitialPath(std::vector<int>(activePath.begin() + routeIndex, activePath.end()));
    std::vector<int> route = dynamicReplanner->updateAndReplan(currentPosition, goalNode, obstacles);
    if (route.empty()) {
        std::cout << "[NAVIGATION] No route from node " << currentPosition << " to goal " << goalNode
                  << " around the reported obstacles; navigation blocked" << std::endl;
        invalidatedRoute.assign(activePath.begin() + routeIndex, activePath.end());
        activePath.resize(routeIndex + 1);  // only the travelled part remains, ending at the current node
        state = NavigationState::BLOCKED;
        return;
    }
    // Keep the part already travelled so the route still reads start -> goal; routeIndex keeps
    // pointing at the current position, which is the first node of the new part.
    std::vector<int> merged(activePath.begin(), activePath.begin() + routeIndex);
    merged.insert(merged.end(), route.begin(), route.end());
    activePath = std::move(merged);
}

void NavigationCore::clearObstacles(const std::vector<int>& nodes) {
    if (!dynamicReplanner) {
        return;
    }
    for (int node : nodes) {
        dynamicReplanner->removePersistentObstacle(node);
    }
}

void NavigationCore::clearAllObstacles() {
    if (dynamicReplanner) {
        dynamicReplanner->clearObstacles();
    }
}

bool NavigationCore::retryNavigation() {
    if (state != NavigationState::BLOCKED || !dynamicReplanner) {
        return false;
    }
    dynamicReplanner->setInitialPath({});
    std::vector<int> route = dynamicReplanner->updateAndReplan(currentPosition, goalNode, {});
    if (route.empty()) {
        return false;
    }
    activePath.insert(activePath.end(), route.begin() + 1, route.end());  // route starts at the current node
    invalidatedRoute.clear();
    state = routeIndex + 1 < activePath.size() ? NavigationState::ACTIVE : NavigationState::COMPLETED;
    dynamicReplanner->setInitialPath(std::vector<int>(activePath.begin() + routeIndex, activePath.end()));
    return true;
}

bool NavigationCore::executeNavigationStep() {
    if (state != NavigationState::ACTIVE) {
        return false;
    }
    ++routeIndex;
    currentPosition = activePath[routeIndex];
    if (routeIndex + 1 == activePath.size()) {
        state = NavigationState::COMPLETED;
    }
    return true;
}

std::string NavigationCore::getNavigationStatus() const {
    std::ostringstream status;
    status << (environment ? navigationStateName(state) : "NO_ENVIRONMENT");
    status << " | mode " << modeName(currentMode);
    if (state == NavigationState::BLOCKED) {
        status << " | at node " << currentPosition << " after " << routeIndex << " steps | no route to goal "
               << goalNode;
    } else if (!activePath.empty()) {
        status << " | at node " << currentPosition << " (" << (routeIndex + 1) << "/" << activePath.size()
               << ") | remaining " << getRemainingDistance();
    }
    return status.str();
}

void NavigationCore::emergencyStop() {
    std::cout << "[NAVIGATION] Emergency stop at node " << currentPosition << std::endl;
    if (state == NavigationState::ACTIVE) {
        state = NavigationState::PAUSED;
    }
}

const Graph* NavigationCore::getCurrentEnvironment() const {
    return environment;
}

#include "autonomous_navigator/NavigationCore.hpp"
#include <algorithm>
#include <iostream>
#include <sstream>
#include <memory>
#include <string>
#include <utility>

// NavigationCore owns the route being followed. Routes come from the GlobalPathPlanner; once
// obstacles are reported, every route is planned through the DynamicReplanner so it avoids them.
// A navigation step moves the current position one node along the route.

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

NavigationCore::NavigationCore(const Graph* graph)
    : environment(graph),
      currentMode(NavigationMode::GLOBAL_ONLY),
      currentPosition(-1),
      navigationActive(false),
      navigationPaused(false) {
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
        navigationActive = false;
        return false;
    }
    activePath = std::move(path);
    currentPosition = activePath.front();
    navigationActive = activePath.size() > 1;
    navigationPaused = false;
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
    navigationActive = false;
    navigationPaused = false;
    activePath.clear();
    currentPosition = -1;
}

void NavigationCore::pauseNavigation() {
    if (navigationActive) {
        navigationActive = false;
        navigationPaused = true;
    }
}

void NavigationCore::resumeNavigation() {
    if (navigationPaused && !activePath.empty() && !isNavigationComplete()) {
        navigationActive = true;
    }
    navigationPaused = false;
}

std::vector<int> NavigationCore::getCurrentPath() const {
    return activePath;
}

int NavigationCore::getCurrentPosition() const {
    return currentPosition;
}

bool NavigationCore::isNavigationComplete() const {
    return !activePath.empty() && currentPosition == activePath.back();
}

double NavigationCore::getRemainingDistance() const {
    auto here = std::find(activePath.begin(), activePath.end(), currentPosition);
    if (here == activePath.end()) {
        return 0.0;
    }
    double remaining = 0.0;
    for (auto it = here; it + 1 != activePath.end(); ++it) {
        remaining += environment->getEdgeWeight(*it, *(it + 1));
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
    if (activePath.empty()) {
        for (int obstacle : obstacles) {
            dynamicReplanner->addPersistentObstacle(obstacle);
        }
        return;
    }
    // Keep the route if it is still clear; otherwise replan from the current position.
    std::vector<int> route = dynamicReplanner->updateAndReplan(currentPosition, activePath.back(), obstacles);
    if (route.empty()) {
        std::cout << "[NAVIGATION] No route around the reported obstacles; navigation halted" << std::endl;
        navigationActive = false;
        return;
    }
    if (route.front() == activePath.front()) {
        activePath = route;
    } else {
        // Keep the part already travelled so the route still reads start -> goal.
        auto here = std::find(activePath.begin(), activePath.end(), currentPosition);
        std::vector<int> merged(activePath.begin(), here);
        merged.insert(merged.end(), route.begin(), route.end());
        activePath = std::move(merged);
    }
}

bool NavigationCore::executeNavigationStep() {
    if (!navigationActive || activePath.empty()) {
        return false;
    }
    auto here = std::find(activePath.begin(), activePath.end(), currentPosition);
    if (here == activePath.end() || here + 1 == activePath.end()) {
        navigationActive = false;
        return false;
    }
    currentPosition = *(here + 1);
    if (currentPosition == activePath.back()) {
        navigationActive = false;  // goal reached
    }
    return true;
}

std::string NavigationCore::getNavigationStatus() const {
    std::ostringstream status;
    if (!environment) {
        status << "NO_ENVIRONMENT";
    } else if (navigationActive) {
        status << "ACTIVE";
    } else if (navigationPaused) {
        status << "PAUSED";
    } else if (isNavigationComplete()) {
        status << "COMPLETED";
    } else {
        status << "IDLE";
    }
    status << " | mode " << modeName(currentMode);
    if (!activePath.empty()) {
        auto here = std::find(activePath.begin(), activePath.end(), currentPosition);
        status << " | at node " << currentPosition << " (" << (here - activePath.begin() + 1) << "/"
               << activePath.size() << ") | remaining " << getRemainingDistance();
    }
    return status.str();
}

void NavigationCore::emergencyStop() {
    std::cout << "[NAVIGATION] Emergency stop at node " << currentPosition << std::endl;
    navigationActive = false;
    navigationPaused = !activePath.empty();
}

const Graph* NavigationCore::getCurrentEnvironment() const {
    return environment;
}

#include "autonomous_navigator/PathExecutor.hpp"
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <functional>
#include <string>
#include <utility>

// PathExecutor state, loading and queries. Starting, stepping, stopping and the emergency stop
// are implemented in emergency_stop_handler.cpp.

// Defined in emergency_stop_handler.cpp.
void detachEmergencyCallback(const PathExecutor* executor);

PathExecutor::PathExecutor(const Graph* graph)
    : environment(graph),
      currentWaypointIndex(0),
      currentState(ExecutionState::IDLE),
      executionStartTime(std::chrono::steady_clock::now()),
      executionSpeed(1.0) {
    if (!environment) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
}

PathExecutor::~PathExecutor() {
    detachEmergencyCallback(this);
}

bool PathExecutor::loadPath(const std::vector<int>& path) {
    if (path.empty()) {
        return false;
    }
    for (size_t i = 0; i < path.size(); ++i) {
        if (!environment->hasNode(path[i]) || (i > 0 && !environment->hasEdge(path[i - 1], path[i]))) {
            std::cout << "[PATH_EXECUTOR] Rejected path: step " << i << " does not follow an edge" << std::endl;
            return false;
        }
    }
    executionPath = path;
    currentWaypointIndex = 0;
    currentState = ExecutionState::IDLE;
    return true;
}

void PathExecutor::pauseExecution() {
    if (currentState == ExecutionState::EXECUTING) {
        currentState = ExecutionState::PAUSED;
    }
}

void PathExecutor::resumeExecution() {
    if (currentState == ExecutionState::PAUSED) {
        currentState = ExecutionState::EXECUTING;
    }
}

ExecutionState PathExecutor::getExecutionState() const {
    return currentState;
}

int PathExecutor::getCurrentWaypoint() const {
    if (executionPath.empty()) {
        return -1;
    }
    // Once the last waypoint is reached the index is past the end; report the final node.
    return executionPath[std::min(currentWaypointIndex, executionPath.size() - 1)];
}

size_t PathExecutor::getRemainingWaypoints() const {
    return currentWaypointIndex >= executionPath.size() ? 0 : executionPath.size() - currentWaypointIndex;
}

double PathExecutor::getExecutionProgress() const {
    if (executionPath.empty()) {
        return 0.0;
    }
    return static_cast<double>(currentWaypointIndex) / static_cast<double>(executionPath.size());
}

void PathExecutor::setWaypointReachedCallback(std::function<void(int)> callback) {
    waypointReachedCallback = std::move(callback);
}

void PathExecutor::setErrorCallback(std::function<void(const std::string&)> callback) {
    errorCallback = std::move(callback);
}

void PathExecutor::setSafetyCheckCallback(std::function<bool(int)> callback) {
    safetyCheckCallback = std::move(callback);
}

void PathExecutor::setExecutionSpeed(double speed) {
    if (speed <= 0.0) {
        throw std::invalid_argument("Execution speed must be positive");
    }
    executionSpeed = speed;
}

double PathExecutor::getExecutionSpeed() const {
    return executionSpeed;
}

std::chrono::duration<double> PathExecutor::getExecutionTime() const {
    if (currentState == ExecutionState::IDLE) {
        return std::chrono::duration<double>(0.0);
    }
    return std::chrono::steady_clock::now() - executionStartTime;
}

void PathExecutor::skipToWaypoint(size_t waypointIndex) {
    if (waypointIndex >= executionPath.size()) {
        throw std::out_of_range("Waypoint index out of range");
    }
    currentWaypointIndex = waypointIndex;
}

bool PathExecutor::insertWaypoint(int nodeId, size_t position) {
    if (position > executionPath.size() || !environment->hasNode(nodeId)) {
        return false;
    }
    const bool joinsPrevious = position == 0 || environment->hasEdge(executionPath[position - 1], nodeId);
    const bool joinsNext = position == executionPath.size() || environment->hasEdge(nodeId, executionPath[position]);
    if (!joinsPrevious || !joinsNext) {
        return false;  // the route would no longer follow edges
    }
    executionPath.insert(executionPath.begin() + static_cast<long>(position), nodeId);
    if (position < currentWaypointIndex) {
        ++currentWaypointIndex;
    }
    return true;
}

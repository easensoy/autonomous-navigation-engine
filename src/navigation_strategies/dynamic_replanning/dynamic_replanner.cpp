#include "navigation_strategies/DynamicReplanner.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <memory>

// DynamicReplanner keeps the current route and a list of blocked nodes, and replans when a newly
// reported obstacle lies on the route. Planning around obstacles runs A* on a copy of the graph
// with the blocked nodes removed, so the route never passes through them. The start is exempt
// (the vehicle is already there); a goal that is itself blocked has no route.

DynamicReplanner::DynamicReplanner(const Graph* graph)
    : environment(graph),
      lastReplanTime(std::chrono::steady_clock::now()),
      replanningInterval(0.0),
      replanningCount(0),
      creationTime(std::chrono::steady_clock::now()) {
    if (!environment) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
    globalPathPlanner = std::make_unique<GlobalPathPlanner>(environment);
}

std::vector<int> DynamicReplanner::planAvoidingObstacles(int startId, int goalId) const {
    if (!environment->hasNode(startId) || !environment->hasNode(goalId)) {
        return {};
    }
    const bool goalBlocked =
        std::find(knownObstacles.begin(), knownObstacles.end(), goalId) != knownObstacles.end();
    if (goalBlocked && goalId != startId) {
        return {};
    }
    bool blocksSomething = false;
    for (int obstacle : knownObstacles) {
        if (obstacle != startId && environment->hasNode(obstacle)) {
            blocksSomething = true;
            break;
        }
    }
    if (!blocksSomething) {
        return globalPathPlanner->planPath(startId, goalId);
    }

    Graph reduced(*environment);
    for (int obstacle : knownObstacles) {
        if (obstacle != startId) {
            reduced.removeNode(obstacle);
        }
    }
    AStar search(&reduced);
    return search.findPath(startId, goalId);
}

bool DynamicReplanner::requiresReplanning(const std::vector<int>& newObstacles) const {
    return std::any_of(newObstacles.begin(), newObstacles.end(), [this](int obstacle) {
        return std::find(currentPath.begin(), currentPath.end(), obstacle) != currentPath.end();
    });
}

void DynamicReplanner::updateObstacleMap(const std::vector<int>& newObstacles) {
    for (int obstacle : newObstacles) {
        if (std::find(knownObstacles.begin(), knownObstacles.end(), obstacle) == knownObstacles.end()) {
            knownObstacles.push_back(obstacle);
        }
    }
}

std::vector<int> DynamicReplanner::updateAndReplan(int currentPosition, int goalId,
                                                   const std::vector<int>& detectedObstacles) {
    updateObstacleMap(detectedObstacles);

    // Keep the remaining part of the current route if it still reaches the goal and is clear.
    auto here = std::find(currentPath.begin(), currentPath.end(), currentPosition);
    const bool routeUsable = here != currentPath.end() && !currentPath.empty() && currentPath.back() == goalId;
    if (routeUsable) {
        const std::vector<int> remaining(here, currentPath.end());
        const bool clear = std::none_of(remaining.begin() + 1, remaining.end(), [this](int node) {
            return std::find(knownObstacles.begin(), knownObstacles.end(), node) != knownObstacles.end();
        });
        if (clear) {
            return remaining;
        }
    }

    std::vector<int> newPath = planAvoidingObstacles(currentPosition, goalId);
    lastReplanTime = std::chrono::steady_clock::now();
    ++replanningCount;
    if (!newPath.empty()) {
        currentPath = newPath;
    }
    return newPath;
}

void DynamicReplanner::setInitialPath(const std::vector<int>& initialPath) {
    currentPath = initialPath;
}

bool DynamicReplanner::shouldReplan(const std::vector<int>& sensorData) const {
    if (requiresReplanning(sensorData)) {
        return true;
    }
    if (replanningInterval > 0.0) {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - lastReplanTime).count();
        return elapsed >= replanningInterval;
    }
    return false;
}

void DynamicReplanner::setReplanningInterval(double intervalSeconds) {
    if (intervalSeconds < 0.0) {
        throw std::invalid_argument("Replanning interval cannot be negative");
    }
    replanningInterval = intervalSeconds;
}

void DynamicReplanner::forceReplan() {
    if (currentPath.size() < 2) {
        return;
    }
    std::vector<int> newPath = planAvoidingObstacles(currentPath.front(), currentPath.back());
    lastReplanTime = std::chrono::steady_clock::now();
    ++replanningCount;
    if (!newPath.empty()) {
        currentPath = newPath;
    }
}

std::vector<int> DynamicReplanner::getCurrentPath() const {
    return currentPath;
}

void DynamicReplanner::addPersistentObstacle(int nodeId) {
    updateObstacleMap({nodeId});
}

void DynamicReplanner::removePersistentObstacle(int nodeId) {
    knownObstacles.erase(std::remove(knownObstacles.begin(), knownObstacles.end(), nodeId), knownObstacles.end());
}

void DynamicReplanner::clearObstacles() {
    knownObstacles.clear();
}

double DynamicReplanner::getReplanningFrequency() const {
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - creationTime).count();
    return elapsed > 0.0 ? static_cast<double>(replanningCount) / elapsed : 0.0;
}

size_t DynamicReplanner::getReplanningCount() const {
    return replanningCount;
}

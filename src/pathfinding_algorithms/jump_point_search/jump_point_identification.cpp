#include "pathfinding_algorithms/JumpPointSearch.hpp"
#include "jps_internals.hpp"
#include <iostream>
#include <unordered_set>
#include <algorithm>
#include <queue>
#include <cmath>
#include <memory>
#include <utility>

void JumpPointSearch::initializeJumpPointIdentification() {
    if (!jumpPointIdentifier) {
        jumpPointIdentifier = std::make_unique<JumpPointIdentifier>(graph, gridWidth, gridHeight);
    }
}

std::pair<int, int> JumpPointSearch::identifyJumpPoint(int x, int y, int dx, int dy, int goalX, int goalY) {
    if (!jumpPointIdentifier) {
        initializeJumpPointIdentification();
    }
    
    return jumpPointIdentifier->identifyJumpPoint(x, y, dx, dy, goalX, goalY);
}

std::vector<std::pair<int, int>> JumpPointSearch::findAllJumpPointsInPath(int startX, int startY, int goalX, int goalY) {
    if (!jumpPointIdentifier) {
        initializeJumpPointIdentification();
    }
    
    return jumpPointIdentifier->findAllJumpPoints(startX, startY, goalX, goalY);
}

bool JumpPointSearch::isNodeJumpPoint(int x, int y, int parentX, int parentY, int goalX, int goalY) {
    if (!jumpPointIdentifier) {
        initializeJumpPointIdentification();
    }
    
    return jumpPointIdentifier->isJumpPoint(x, y, parentX, parentY, goalX, goalY);
}

std::vector<std::pair<int, int>> JumpPointSearch::identifyForcedNeighborsAtPosition(int x, int y, int dx, int dy) {
    if (!jumpPointIdentifier) {
        initializeJumpPointIdentification();
    }
    
    return jumpPointIdentifier->checkForForcedNeighbors(x, y, dx, dy);
}

void JumpPointSearch::analyzeJumpPointDistribution() {
    if (jumpPointIdentifier) {
        jumpPointIdentifier->analyzeJumpPointDistribution();
    }
}

double JumpPointSearch::calculateJumpPointDensity() {
    return jumpPointIdentifier ? jumpPointIdentifier->calculateJumpPointDensity() : 0.0;
}

void JumpPointSearch::optimizeJumpPointIdentification(bool enable) {
    if (!jumpPointIdentifier) {
        initializeJumpPointIdentification();
    }
    
    jumpPointIdentifier->optimizeJumpPointIdentification(enable);
}

void JumpPointSearch::clearJumpPointCache() {
    if (jumpPointIdentifier) {
        jumpPointIdentifier->clearJumpPointCache();
    }
}

std::vector<std::pair<int, int>> JumpPointSearch::getJumpPointsInRegion(int minX, int minY, int maxX, int maxY) {
    if (!jumpPointIdentifier) {
        initializeJumpPointIdentification();
    }
    
    return jumpPointIdentifier->getJumpPointsInRegion(minX, minY, maxX, maxY);
}
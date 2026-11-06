#include "pathfinding_algorithms/JumpPointSearch.hpp"
#include "jps_internals.hpp"
#include <iostream>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

void JumpPointSearch::initializeGridPruning() {
    if (!pruningManager) {
        pruningManager = std::make_unique<GridPruningManager>(graph, gridWidth, gridHeight);
    }
}

std::vector<std::pair<int, int>> JumpPointSearch::pruneNeighbors(int x, int y, int parentX, int parentY) {
    if (!pruningManager) {
        initializeGridPruning();
    }
    
    return pruningManager->pruneNeighbors(x, y, parentX, parentY);
}

std::vector<std::pair<int, int>> JumpPointSearch::pruneDiagonalMovement(int x, int y, int dx, int dy) {
    if (!pruningManager) {
        initializeGridPruning();
    }
    
    return pruningManager->pruneDiagonalNeighbors(x, y, dx, dy);
}

std::vector<std::pair<int, int>> JumpPointSearch::pruneCardinalMovement(int x, int y, int dx, int dy) {
    if (!pruningManager) {
        initializeGridPruning();
    }
    
    if (dx != 0) {
        return pruningManager->pruneHorizontalNeighbors(x, y, dx);
    } else {
        return pruningManager->pruneVerticalNeighbors(x, y, dy);
    }
}

std::vector<std::pair<int, int>> JumpPointSearch::identifyForcedNeighbors(int x, int y, int dx, int dy) {
    if (!pruningManager) {
        initializeGridPruning();
    }
    
    return pruningManager->identifyForcedNeighbors(x, y, dx, dy);
}

bool JumpPointSearch::shouldPruneNode(int x, int y, int parentX, int parentY, int goalX, int goalY) {
    if (!pruningManager) {
        initializeGridPruning();
    }
    
    return pruningManager->shouldPruneNode(x, y, parentX, parentY, goalX, goalY);
}

double JumpPointSearch::calculatePruningEfficiency() {
    return pruningManager ? pruningManager->calculatePruningEfficiency() : 0.0;
}

void JumpPointSearch::enableSymmetryBreaking(bool enable) {
    if (!pruningManager) {
        initializeGridPruning();
    }
    
    pruningManager->enableSymmetryBreaking(enable);
}

void JumpPointSearch::resetPruningState() {
    if (pruningManager) {
        pruningManager->resetPruningState();
    }
}

void JumpPointSearch::analyzePruningPatterns() {
    if (pruningManager) {
        pruningManager->analyzePruningPatterns();
    }
}
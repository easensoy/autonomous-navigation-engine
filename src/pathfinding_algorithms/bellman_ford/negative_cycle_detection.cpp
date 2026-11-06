#include "pathfinding_algorithms/BellmanFord.hpp"
#include "bellman_ford_internals.hpp"
#include <iostream>
#include <limits>
#include <unordered_set>
#include <memory>
#include <unordered_map>

bool BellmanFord::initializeNegativeCycleDetection() {
    if (!cycleDetector) {
        cycleDetector = std::make_unique<NegativeCycleDetector>(graph);
        return true;
    }
    return false;
}

bool BellmanFord::hasNegativeCycleInDistances(const std::unordered_map<int, double>& distances,
                                             const std::unordered_map<int, int>& predecessors) {
    if (!cycleDetector) {
        initializeNegativeCycleDetection();
    }
    
    return cycleDetector->hasNegativeCycle(distances, predecessors);
}

std::vector<int> BellmanFord::getLastDetectedNegativeCycle() const {
    return cycleDetector ? cycleDetector->getLastDetectedCycle() : std::vector<int>();
}

std::vector<int> BellmanFord::findAllNegativeCycles() {
    if (!cycleDetector) {
        initializeNegativeCycleDetection();
    }
    
    return cycleDetector->findAllNegativeCycles();
}

bool BellmanFord::isNodeInNegativeCycle(int nodeId) {
    if (!cycleDetector) {
        initializeNegativeCycleDetection();
    }
    
    return cycleDetector->isNodeInNegativeCycle(nodeId);
}

void BellmanFord::enableNegativeCycleDetection(bool enabled) {
    if (!cycleDetector) {
        initializeNegativeCycleDetection();
    }
    
    cycleDetector->setDetectionEnabled(enabled);
}

double BellmanFord::calculateNegativeCycleWeight(const std::vector<int>& cycle) const {
    if (!cycleDetector) {
        return 0.0;
    }
    
    return cycleDetector->calculateCycleWeight(cycle);
}
#include "pathfinding_algorithms/BellmanFord.hpp"
#include "bellman_ford_internals.hpp"
#include <iostream>
#include <limits>
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>

void BellmanFord::initializeEdgeRelaxation(const std::vector<int>& nodeIds, int startNode) {
    if (!relaxationManager) {
        relaxationManager = std::make_unique<EdgeRelaxationManager>();
    }
    
    relaxationManager->initializeDistances(nodeIds, startNode);
}

bool BellmanFord::relaxSingleEdge(int fromNode, int toNode, double edgeWeight) {
    if (!relaxationManager) {
        throw std::runtime_error("Edge relaxation not initialized");
    }
    
    return relaxationManager->relaxEdge(fromNode, toNode, edgeWeight);
}

size_t BellmanFord::performFullEdgeRelaxationPass() {
    if (!relaxationManager) {
        throw std::runtime_error("Edge relaxation not initialized");
    }
    
    return relaxationManager->performFullRelaxationPass(graph);
}

std::unordered_map<int, double> BellmanFord::getCurrentDistancesFromRelaxation() const {
    return relaxationManager ? relaxationManager->getDistances() : std::unordered_map<int, double>();
}

std::unordered_map<int, int> BellmanFord::getCurrentPredecessorsFromRelaxation() const {
    return relaxationManager ? relaxationManager->getPredecessors() : std::unordered_map<int, int>();
}

size_t BellmanFord::getEdgeRelaxationCount() const {
    return relaxationManager ? relaxationManager->getRelaxationCount() : 0;
}

size_t BellmanFord::getTotalEdgesProcessed() const {
    return relaxationManager ? relaxationManager->getTotalEdgeCount() : 0;
}

double BellmanFord::getEdgeRelaxationRatio() const {
    return relaxationManager ? relaxationManager->getRelaxationRatio() : 0.0;
}

void BellmanFord::setEdgeRelaxationLogging(bool enabled) {
    if (!relaxationManager) {
        relaxationManager = std::make_unique<EdgeRelaxationManager>();
    }
    
    relaxationManager->setDetailedLogging(enabled);
}

void BellmanFord::resetEdgeRelaxationStatistics() {
    if (relaxationManager) {
        relaxationManager->resetStatistics();
    }
}

std::vector<std::pair<int, double>> BellmanFord::getReachableNodesWithDistances() const {
    return relaxationManager ? relaxationManager->getReachableNodes() : 
           std::vector<std::pair<int, double>>();
}

void BellmanFord::printEdgeRelaxationStatistics() const {
    if (relaxationManager) {
        relaxationManager->printRelaxationStatistics();
    } else {
        std::cout << "[BELLMAN-FORD] Edge relaxation manager not initialized" << std::endl;
    }
}
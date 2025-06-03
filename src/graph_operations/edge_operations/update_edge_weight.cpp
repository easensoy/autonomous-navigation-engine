#include "core/Graph.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

bool updateEdgeWeight(Graph* graph, int fromId, int toId, double newWeight) {
    if (!graph) {
        std::cout << "[GRAPH] Cannot update edge weight - null graph pointer" << std::endl;
        return false;
    }
    
    std::cout << "[GRAPH] Attempting to update edge weight: " << fromId << " -> " << toId 
              << " to " << newWeight << std::endl;
    
    // Validate that both nodes exist
    if (!graph->hasNode(fromId)) {
        std::cout << "[GRAPH] Cannot update edge - source node " << fromId << " does not exist" << std::endl;
        return false;
    }
    
    if (!graph->hasNode(toId)) {
        std::cout << "[GRAPH] Cannot update edge - target node " << toId << " does not exist" << std::endl;
        return false;
    }
    
    // Validate weight
    if (newWeight < 0) {
        std::cout << "[GRAPH] Cannot update edge - negative weight not allowed: " << newWeight << std::endl;
        return false;
    }
    
    if (std::isnan(newWeight) || std::isinf(newWeight)) {
        std::cout << "[GRAPH] Cannot update edge - invalid weight value: " << newWeight << std::endl;
        return false;
    }
    
    // Check if edge exists
    if (!graph->hasEdge(fromId, toId)) {
        std::cout << "[GRAPH] Cannot update edge - edge " << fromId << " -> " << toId << " does not exist" << std::endl;
        return false;
    }
    
    // Get the current edge to check if it's bidirectional
    const std::vector<Edge>& edges = graph->getEdgesFrom(fromId);
    auto it = std::find_if(edges.begin(), edges.end(),
        [toId](const Edge& edge) { return edge.getToNode() == toId; });
    
    if (it == edges.end()) {
        std::cout << "[GRAPH] Edge not found in adjacency list" << std::endl;
        return false;
    }
    
    double oldWeight = it->getWeight();
    bool isBidirectional = it->isBidirectional();
    
    // Remove the old edge and add new one with updated weight
    graph->removeEdge(fromId, toId);
    graph->addEdge(fromId, toId, newWeight, isBidirectional);
    
    std::cout << "[GRAPH] Successfully updated edge weight " << fromId << " -> " << toId 
              << " from " << oldWeight << " to " << newWeight << std::endl;
    
    return true;
}

// Function to update weights of multiple edges
bool updateMultipleEdgeWeights(Graph* graph, const std::vector<std::tuple<int, int, double>>& updates) {
    if (!graph) {
        std::cout << "[GRAPH] Cannot update edge weights - null graph pointer" << std::endl;
        return false;
    }
    
    std::cout << "[GRAPH] Updating weights for " << updates.size() << " edges" << std::endl;
    
    size_t successfulUpdates = 0;
    for (const auto& [fromId, toId, newWeight] : updates) {
        if (updateEdgeWeight(graph, fromId, toId, newWeight)) {
            successfulUpdates++;
        }
    }
    
    std::cout << "[GRAPH] Successfully updated " << successfulUpdates 
              << " out of " << updates.size() << " edge weights" << std::endl;
    
    return successfulUpdates == updates.size();
}

// Function to scale all edge weights by a factor
bool scaleAllEdgeWeights(Graph* graph, double scaleFactor) {
    if (!graph) {
        std::cout << "[GRAPH] Cannot scale edge weights - null graph pointer" << std::endl;
        return false;
    }
    
    if (scaleFactor <= 0) {
        std::cout << "[GRAPH] Cannot scale edge weights - invalid scale factor: " << scaleFactor << std::endl;
        return false;
    }
    
    std::cout << "[GRAPH] Scaling all edge weights by factor " << scaleFactor << std::endl;
    
    std::vector<std::tuple<int, int, double>> updates;
    std::vector<int> nodeIds = graph->getAllNodeIds();
    
    // Collect all edges and their new weights
    for (int nodeId : nodeIds) {
        const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
        for (const Edge& edge : edges) {
            double newWeight = edge.getWeight() * scaleFactor;
            updates.emplace_back(edge.getFromNode(), edge.getToNode(), newWeight);
        }
    }
    
    return updateMultipleEdgeWeights(graph, updates);
}

// Function to normalize edge weights to a specific range
bool normalizeEdgeWeights(Graph* graph, double minWeight, double maxWeight) {
    if (!graph) {
        std::cout << "[GRAPH] Cannot normalize edge weights - null graph pointer" << std::endl;
        return false;
    }
    
    if (minWeight >= maxWeight) {
        std::cout << "[GRAPH] Cannot normalize - invalid weight range: [" 
                  << minWeight << ", " << maxWeight << "]" << std::endl;
        return false;
    }
    
    std::cout << "[GRAPH] Normalizing edge weights to range [" << minWeight 
              << ", " << maxWeight << "]" << std::endl;
    
    // Find current min and max weights
    double currentMin = std::numeric_limits<double>::infinity();
    double currentMax = -std::numeric_limits<double>::infinity();
    
    std::vector<int> nodeIds = graph->getAllNodeIds();
    for (int nodeId : nodeIds) {
        const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
        for (const Edge& edge : edges) {
            double weight = edge.getWeight();
            currentMin = std::min(currentMin, weight);
            currentMax = std::max(currentMax, weight);
        }
    }
    
    if (currentMin == currentMax) {
        std::cout << "[GRAPH] All edges have the same weight (" << currentMin 
                  << ") - setting all to " << minWeight << std::endl;
        
        std::vector<std::tuple<int, int, double>> updates;
        for (int nodeId : nodeIds) {
            const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
            for (const Edge& edge : edges) {
                updates.emplace_back(edge.getFromNode(), edge.getToNode(), minWeight);
            }
        }
        return updateMultipleEdgeWeights(graph, updates);
    }
    
    // Normalize weights
    std::vector<std::tuple<int, int, double>> updates;
    double currentRange = currentMax - currentMin;
    double targetRange = maxWeight - minWeight;
    
    for (int nodeId : nodeIds) {
        const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
        for (const Edge& edge : edges) {
            double normalizedWeight = minWeight + ((edge.getWeight() - currentMin) / currentRange) * targetRange;
            updates.emplace_back(edge.getFromNode(), edge.getToNode(), normalizedWeight);
        }
    }
    
    std::cout << "[GRAPH] Normalizing from range [" << currentMin << ", " << currentMax 
              << "] to [" << minWeight << ", " << maxWeight << "]" << std::endl;
    
    return updateMultipleEdgeWeights(graph, updates);
}

// Function to update weights based on Euclidean distance between nodes
bool updateWeightsToEuclideanDistance(Graph* graph, double scaleFactor = 1.0) {
    if (!graph) {
        std::cout << "[GRAPH] Cannot update weights - null graph pointer" << std::endl;
        return false;
    }
    
    std::cout << "[GRAPH] Updating edge weights to Euclidean distances (scale: " 
              << scaleFactor << ")" << std::endl;
    
    std::vector<std::tuple<int, int, double>> updates;
    std::vector<int> nodeIds = graph->getAllNodeIds();
    
    for (int nodeId : nodeIds) {
        const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
        for (const Edge& edge : edges) {
            const Node& fromNode = graph->getNode(edge.getFromNode());
            const Node& toNode = graph->getNode(edge.getToNode());
            
            double euclideanDistance = fromNode.euclideanDistance(toNode);
            double newWeight = euclideanDistance * scaleFactor;
            
            updates.emplace_back(edge.getFromNode(), edge.getToNode(), newWeight);
        }
    }
    
    return updateMultipleEdgeWeights(graph, updates);
}

// Function to apply a weight transformation function to all edges
bool transformEdgeWeights(Graph* graph, std::function<double(double)> transformFunction) {
    if (!graph) {
        std::cout << "[GRAPH] Cannot transform edge weights - null graph pointer" << std::endl;
        return false;
    }
    
    if (!transformFunction) {
        std::cout << "[GRAPH] Cannot transform edge weights - null transform function" << std::endl;
        return false;
    }
    
    std::cout << "[GRAPH] Applying weight transformation to all edges" << std::endl;
    
    std::vector<std::tuple<int, int, double>> updates;
    std::vector<int> nodeIds = graph->getAllNodeIds();
    
    for (int nodeId : nodeIds) {
        const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
        for (const Edge& edge : edges) {
            double currentWeight = edge.getWeight();
            double newWeight = transformFunction(currentWeight);
            
            // Validate transformed weight
            if (newWeight >= 0 && !std::isnan(newWeight) && !std::isinf(newWeight)) {
                updates.emplace_back(edge.getFromNode(), edge.getToNode(), newWeight);
            } else {
                std::cout << "[GRAPH] Skipping invalid transformed weight for edge " 
                          << edge.getFromNode() << " -> " << edge.getToNode() 
                          << ": " << newWeight << std::endl;
            }
        }
    }
    
    return updateMultipleEdgeWeights(graph, updates);
}
#pragma once
#include "../core/Graph.hpp"
#include <vector>
#include <utility>

class EdgeManager {
private:
    Graph* graph;
    
    bool validateEdgeAddition(int fromId, int toId) const;
    void updateConnectivity(int fromId, int toId, bool added);

public:
    explicit EdgeManager(Graph* environment);
    
    bool addEdge(int fromId, int toId, double weight, bool bidirectional = true);
    bool removeEdge(int fromId, int toId);
    bool updateEdgeWeight(int fromId, int toId, double newWeight);
    
    std::vector<std::pair<int, int>> findShortestEdges(int count) const;
    std::vector<std::pair<int, int>> findLongestEdges(int count) const;
    std::vector<std::pair<int, int>> findRedundantEdges() const;
    
    void removeRedundantEdges();
    void addMissingConnections(double maxDistance);
    void optimizeEdgeWeights();
    
    double getTotalEdgeWeight() const;
    double getAverageEdgeWeight() const;
    std::pair<int, int> getHeaviestEdge() const;
    
    bool createSpanningTree();
    std::vector<std::pair<int, int>> getMinimumSpanningTree() const;
    void ensureConnectivity();
    
    size_t getEdgeCount() const;
    bool validateEdgeConsistency() const;
};
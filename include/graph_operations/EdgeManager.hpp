#pragma once
#include "core/Graph.hpp"
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
    
    
    double getTotalEdgeWeight() const;
    double getAverageEdgeWeight() const;
    std::pair<int, int> getHeaviestEdge() const;
    
    // Minimum spanning forest by Kruskal's algorithm, one tree per connected component, as
    // (smaller id, larger id) pairs in the order chosen (by weight, then ids). Edges are treated as
    // undirected: two nodes joined in either direction are one candidate, weighted by the cheaper one.
    std::vector<std::pair<int, int>> getMinimumSpanningTree() const;
    
    size_t getEdgeCount() const;
    bool validateEdgeConsistency() const;
};
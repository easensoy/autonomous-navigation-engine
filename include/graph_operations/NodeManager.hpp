#pragma once
#include "core/Graph.hpp"
#include <vector>
#include <string>

class NodeManager {
private:
    Graph* graph;
    int nextNodeId;
    
    bool validateNodePlacement(double x, double y) const;
    int generateUniqueNodeId();

public:
    explicit NodeManager(Graph* environment);
    
    int addNode(const std::string& name, double x, double y);
    int addNodeWithId(int nodeId, const std::string& name, double x, double y);
    bool removeNode(int nodeId);
    // Moves a node. Returns false, changing nothing, if the node is unknown or the position is not
    // finite or already holds another node. Edge weights are not recomputed.
    bool moveNode(int nodeId, double newX, double newY);
    
    // Renames a node; false if the node is unknown.
    bool renameNode(int nodeId, const std::string& newName);
    std::vector<int> findNodesInRadius(double centerX, double centerY, double radius) const;
    int findNearestNode(double x, double y) const;
    
    std::vector<int> getIsolatedNodes() const;
    std::vector<int> getHighDegreeNodes(int minDegree) const;
    
    std::vector<int> clusterNodes(double maxDistance) const;
    
    size_t getNodeCount() const;
    std::vector<int> getAllNodeIds() const;
    bool validateNodeConsistency() const;
};
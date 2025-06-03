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
    bool moveNode(int nodeId, double newX, double newY);
    
    bool renameNode(int nodeId, const std::string& newName);
    std::vector<int> findNodesInRadius(double centerX, double centerY, double radius) const;
    int findNearestNode(double x, double y) const;
    
    std::vector<int> getIsolatedNodes() const;
    std::vector<int> getHighDegreeNodes(int minDegree) const;
    void optimizeNodePositions();
    
    bool mergeNodes(int nodeId1, int nodeId2);
    std::vector<int> clusterNodes(double maxDistance) const;
    void redistributeNodes(double minSpacing);
    
    size_t getNodeCount() const;
    std::vector<int> getAllNodeIds() const;
    bool validateNodeConsistency() const;
};
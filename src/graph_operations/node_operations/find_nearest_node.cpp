#include "core/Graph.hpp"
#include <iostream>
#include <limits>
#include <algorithm>
#include <queue>
#include <cmath>

class NearestNodeFinder {
private:
    const Graph* graph;
    bool enableLogging;
    
public:
    explicit NearestNodeFinder(const Graph* environment) 
        : graph(environment), enableLogging(false) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
    }
    
    int findNearestNode(double x, double y) {
        if (enableLogging) {
            std::cout << "[NEAREST_NODE] Finding nearest node to (" << x << "," << y << ")" << std::endl;
        }
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        if (nodeIds.empty()) {
            if (enableLogging) {
                std::cout << "[NEAREST_NODE] No nodes in graph" << std::endl;
            }
            return -1;
        }
        
        int nearestId = -1;
        double minDistance = std::numeric_limits<double>::infinity();
        
        for (int nodeId : nodeIds) {
            const Node& node = graph->getNode(nodeId);
            double distance = calculateDistance(x, y, node.getX(), node.getY());
            
            if (distance < minDistance) {
                minDistance = distance;
                nearestId = nodeId;
            }
        }
        
        if (enableLogging) {
            std::cout << "[NEAREST_NODE] Found nearest node " << nearestId 
                      << " at distance " << minDistance << std::endl;
        }
        
        return nearestId;
    }
    
    std::vector<int> findKNearestNodes(double x, double y, int k) {
        if (enableLogging) {
            std::cout << "[NEAREST_NODE] Finding " << k << " nearest nodes to (" << x << "," << y << ")" << std::endl;
        }
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        if (nodeIds.empty()) {
            return {};
        }
        
        // Use priority queue to maintain k nearest nodes
        std::priority_queue<std::pair<double, int>> maxHeap;
        
        for (int nodeId : nodeIds) {
            const Node& node = graph->getNode(nodeId);
            double distance = calculateDistance(x, y, node.getX(), node.getY());
            
            if (static_cast<int>(maxHeap.size()) < k) {
                maxHeap.push({distance, nodeId});
            } else if (distance < maxHeap.top().first) {
                maxHeap.pop();
                maxHeap.push({distance, nodeId});
            }
        }
        
        // Extract results in ascending order of distance
        std::vector<int> result;
        std::vector<std::pair<double, int>> temp;
        
        while (!maxHeap.empty()) {
            temp.push_back(maxHeap.top());
            maxHeap.pop();
        }
        
        std::reverse(temp.begin(), temp.end());
        for (const auto& pair : temp) {
            result.push_back(pair.second);
        }
        
        if (enableLogging) {
            std::cout << "[NEAREST_NODE] Found " << result.size() << " nearest nodes" << std::endl;
        }
        
        return result;
    }
    
    std::vector<int> findNodesWithinRadius(double x, double y, double radius) {
        if (enableLogging) {
            std::cout << "[NEAREST_NODE] Finding nodes within radius " << radius 
                      << " of (" << x << "," << y << ")" << std::endl;
        }
        
        std::vector<int> result;
        std::vector<int> nodeIds = graph->getAllNodeIds();
        
        for (int nodeId : nodeIds) {
            const Node& node = graph->getNode(nodeId);
            double distance = calculateDistance(x, y, node.getX(), node.getY());
            
            if (distance <= radius) {
                result.push_back(nodeId);
            }
        }
        
        // Sort by distance
        std::sort(result.begin(), result.end(), 
                 [this, x, y](int a, int b) {
                     const Node& nodeA = graph->getNode(a);
                     const Node& nodeB = graph->getNode(b);
                     double distA = calculateDistance(x, y, nodeA.getX(), nodeA.getY());
                     double distB = calculateDistance(x, y, nodeB.getX(), nodeB.getY());
                     return distA < distB;
                 });
        
        if (enableLogging) {
            std::cout << "[NEAREST_NODE] Found " << result.size() << " nodes within radius" << std::endl;
        }
        
        return result;
    }
    
    int findNearestNodeToNode(int referenceNodeId) {
        if (!graph->hasNode(referenceNodeId)) {
            if (enableLogging) {
                std::cout << "[NEAREST_NODE] Reference node " << referenceNodeId << " not found" << std::endl;
            }
            return -1;
        }
        
        const Node& refNode = graph->getNode(referenceNodeId);
        return findNearestNodeExcluding(refNode.getX(), refNode.getY(), {referenceNodeId});
    }
    
    int findNearestNodeExcluding(double x, double y, const std::vector<int>& excludeIds) {
        if (enableLogging) {
            std::cout << "[NEAREST_NODE] Finding nearest node excluding " << excludeIds.size() 
                      << " nodes" << std::endl;
        }
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        if (nodeIds.empty()) {
            return -1;
        }
        
        int nearestId = -1;
        double minDistance = std::numeric_limits<double>::infinity();
        
        for (int nodeId : nodeIds) {
            // Skip excluded nodes
            if (std::find(excludeIds.begin(), excludeIds.end(), nodeId) != excludeIds.end()) {
                continue;
            }
            
            const Node& node = graph->getNode(nodeId);
            double distance = calculateDistance(x, y, node.getX(), node.getY());
            
            if (distance < minDistance) {
                minDistance = distance;
                nearestId = nodeId;
            }
        }
        
        return nearestId;
    }
    
    std::pair<int, double> findNearestNodeWithDistance(double x, double y) {
        int nearestId = findNearestNode(x, y);
        
        if (nearestId == -1) {
            return {-1, std::numeric_limits<double>::infinity()};
        }
        
        const Node& node = graph->getNode(nearestId);
        double distance = calculateDistance(x, y, node.getX(), node.getY());
        
        return {nearestId, distance};
    }
    
    void setLogging(bool enable) {
        enableLogging = enable;
    }
    
private:
    double calculateDistance(double x1, double y1, double x2, double y2) const {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return std::sqrt(dx * dx + dy * dy);
    }
};

// Global nearest node finder
static std::unique_ptr<NearestNodeFinder> g_nearestFinder;

void initializeNearestNodeFinder(const Graph* graph) {
    g_nearestFinder = std::make_unique<NearestNodeFinder>(graph);
}

int findNearestNode(const Graph* graph, double x, double y) {
    if (!g_nearestFinder) {
        g_nearestFinder = std::make_unique<NearestNodeFinder>(graph);
    }
    return g_nearestFinder->findNearestNode(x, y);
}

std::vector<int> findKNearestNodes(const Graph* graph, double x, double y, int k) {
    if (!g_nearestFinder) {
        g_nearestFinder = std::make_unique<NearestNodeFinder>(graph);
    }
    return g_nearestFinder->findKNearestNodes(x, y, k);
}

std::vector<int> findNodesInRadius(const Graph* graph, double x, double y, double radius) {
    if (!g_nearestFinder) {
        g_nearestFinder = std::make_unique<NearestNodeFinder>(graph);
    }
    return g_nearestFinder->findNodesWithinRadius(x, y, radius);
}

int findNearestNodeToNode(const Graph* graph, int referenceNodeId) {
    if (!g_nearestFinder) {
        g_nearestFinder = std::make_unique<NearestNodeFinder>(graph);
    }
    return g_nearestFinder->findNearestNodeToNode(referenceNodeId);
}
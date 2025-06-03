#include "core/Graph.hpp"
#include <iostream>
#include <stdexcept>
#include <algorithm>

class NodeAdditionManager {
private:
    Graph* graph;
    int nextAutoId;
    bool enableValidation;
    double minDistanceBetweenNodes;
    
public:
    explicit NodeAdditionManager(Graph* environment) 
        : graph(environment), nextAutoId(1000), enableValidation(true), 
          minDistanceBetweenNodes(0.1) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        // Find the highest existing node ID to avoid conflicts
        std::vector<int> existingIds = graph->getAllNodeIds();
        if (!existingIds.empty()) {
            nextAutoId = *std::max_element(existingIds.begin(), existingIds.end()) + 1;
        }
    }
    
    int addNodeWithAutoId(const std::string& name, double x, double y) {
        std::cout << "[NODE_ADD] Adding node with auto-generated ID at (" << x << "," << y << ")" << std::endl;
        
        if (enableValidation && !validateNodePlacement(x, y)) {
            std::cout << "[NODE_ADD] Node placement validation failed" << std::endl;
            return -1;
        }
        
        int nodeId = nextAutoId++;
        
        try {
            graph->addNode(nodeId, name.empty() ? ("node_" + std::to_string(nodeId)) : name, x, y);
            std::cout << "[NODE_ADD] Successfully added node " << nodeId << " at (" << x << "," << y << ")" << std::endl;
            return nodeId;
        } catch (const std::exception& e) {
            std::cout << "[NODE_ADD] Failed to add node: " << e.what() << std::endl;
            nextAutoId--; // Revert ID increment on failure
            return -1;
        }
    }
    
    bool addNodeWithSpecificId(int nodeId, const std::string& name, double x, double y) {
        std::cout << "[NODE_ADD] Adding node with specific ID " << nodeId << " at (" << x << "," << y << ")" << std::endl;
        
        if (graph->hasNode(nodeId)) {
            std::cout << "[NODE_ADD] Node with ID " << nodeId << " already exists" << std::endl;
            return false;
        }
        
        if (enableValidation && !validateNodePlacement(x, y)) {
            std::cout << "[NODE_ADD] Node placement validation failed" << std::endl;
            return false;
        }
        
        try {
            graph->addNode(nodeId, name.empty() ? ("node_" + std::to_string(nodeId)) : name, x, y);
            
            // Update nextAutoId if necessary
            if (nodeId >= nextAutoId) {
                nextAutoId = nodeId + 1;
            }
            
            std::cout << "[NODE_ADD] Successfully added node " << nodeId << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cout << "[NODE_ADD] Failed to add node: " << e.what() << std::endl;
            return false;
        }
    }
    
    std::vector<int> addMultipleNodes(const std::vector<std::tuple<std::string, double, double>>& nodeData) {
        std::cout << "[NODE_ADD] Adding batch of " << nodeData.size() << " nodes" << std::endl;
        
        std::vector<int> addedNodeIds;
        addedNodeIds.reserve(nodeData.size());
        
        for (const auto& [name, x, y] : nodeData) {
            int nodeId = addNodeWithAutoId(name, x, y);
            if (nodeId != -1) {
                addedNodeIds.push_back(nodeId);
            }
        }
        
        std::cout << "[NODE_ADD] Successfully added " << addedNodeIds.size() 
                  << " out of " << nodeData.size() << " nodes" << std::endl;
        
        return addedNodeIds;
    }
    
    bool addNodeFromCopy(const Node& sourceNode, const std::string& newName = "") {
        std::cout << "[NODE_ADD] Creating node copy from existing node " << sourceNode.getId() << std::endl;
        
        std::string nodeName = newName.empty() ? (sourceNode.getName() + "_copy") : newName;
        
        return addNodeWithSpecificId(nextAutoId, nodeName, sourceNode.getX(), sourceNode.getY()) != -1;
    }
    
    void setValidation(bool enable) {
        enableValidation = enable;
        std::cout << "[NODE_ADD] Node validation " << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void setMinimumDistance(double minDist) {
        minDistanceBetweenNodes = minDist;
        std::cout << "[NODE_ADD] Minimum distance between nodes set to " << minDist << std::endl;
    }
    
private:
    bool validateNodePlacement(double x, double y) {
        if (!enableValidation) return true;
        
        // Check coordinate bounds
        static const double MAX_COORDINATE = 10000.0;
        static const double MIN_COORDINATE = -10000.0;
        
        if (x < MIN_COORDINATE || x > MAX_COORDINATE || y < MIN_COORDINATE || y > MAX_COORDINATE) {
            std::cout << "[NODE_ADD] Coordinates out of valid range" << std::endl;
            return false;
        }
        
        // Check minimum distance to existing nodes
        if (minDistanceBetweenNodes > 0) {
            std::vector<int> existingNodes = graph->getAllNodeIds();
            for (int nodeId : existingNodes) {
                const Node& existing = graph->getNode(nodeId);
                double distance = std::sqrt(std::pow(x - existing.getX(), 2) + std::pow(y - existing.getY(), 2));
                
                if (distance < minDistanceBetweenNodes) {
                    std::cout << "[NODE_ADD] Too close to existing node " << nodeId 
                              << " (distance: " << distance << ")" << std::endl;
                    return false;
                }
            }
        }
        
        return true;
    }
};

// Global node addition functions
static std::unique_ptr<NodeAdditionManager> g_nodeManager;

void initializeNodeAddition(Graph* graph) {
    g_nodeManager = std::make_unique<NodeAdditionManager>(graph);
}

int addNode(Graph* graph, const std::string& name, double x, double y) {
    if (!g_nodeManager) {
        g_nodeManager = std::make_unique<NodeAdditionManager>(graph);
    }
    return g_nodeManager->addNodeWithAutoId(name, x, y);
}

bool addNodeWithId(Graph* graph, int nodeId, const std::string& name, double x, double y) {
    if (!g_nodeManager) {
        g_nodeManager = std::make_unique<NodeAdditionManager>(graph);
    }
    return g_nodeManager->addNodeWithSpecificId(nodeId, name, x, y);
}

std::vector<int> addNodesFromCoordinates(Graph* graph, const std::vector<std::pair<double, double>>& coordinates) {
    if (!g_nodeManager) {
        g_nodeManager = std::make_unique<NodeAdditionManager>(graph);
    }
    
    std::vector<std::tuple<std::string, double, double>> nodeData;
    for (size_t i = 0; i < coordinates.size(); ++i) {
        nodeData.emplace_back("coord_" + std::to_string(i), coordinates[i].first, coordinates[i].second);
    }
    
    return g_nodeManager->addMultipleNodes(nodeData);
}
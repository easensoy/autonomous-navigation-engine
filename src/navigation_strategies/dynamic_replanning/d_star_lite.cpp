#include "navigation_strategies/DynamicReplanner.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <limits>
#include <algorithm>

class DStarLite {
private:
    struct DStarNode {
        int nodeId;
        double gValue;
        double rhsValue;
        double fValue;
        bool inQueue;
        
        DStarNode(int id = -1) : nodeId(id), gValue(std::numeric_limits<double>::infinity()),
                                rhsValue(std::numeric_limits<double>::infinity()), 
                                fValue(std::numeric_limits<double>::infinity()), inQueue(false) {}
        
        bool operator>(const DStarNode& other) const {
            if (fValue != other.fValue) return fValue > other.fValue;
            return gValue > other.gValue;
        }
    };
    
    const Graph* graph;
    std::unordered_map<int, DStarNode> nodes;
    std::priority_queue<DStarNode, std::vector<DStarNode>, std::greater<DStarNode>> priorityQueue;
    std::unordered_map<std::pair<int, int>, double, std::hash<std::pair<int, int>>> edgeCosts;
    std::unordered_set<std::pair<int, int>, std::hash<std::pair<int, int>>> changedEdges;
    
    int startNode;
    int goalNode;
    double keyModifier;
    
    std::pair<double, double> calculateKey(int nodeId) const {
        auto& node = nodes.at(nodeId);
        double heuristic = calculateHeuristic(nodeId, startNode);
        double minG = std::min(node.gValue, node.rhsValue);
        return {minG + heuristic + keyModifier, minG};
    }
    
    double calculateHeuristic(int from, int to) const {
        if (!graph->hasNode(from) || !graph->hasNode(to)) {
            return std::numeric_limits<double>::infinity();
        }
        
        const Node& fromNode = graph->getNode(from);
        const Node& toNode = graph->getNode(to);
        return fromNode.euclideanDistance(toNode);
    }
    
    double getEdgeCost(int from, int to) const {
        auto key = std::make_pair(from, to);
        auto it = edgeCosts.find(key);
        if (it != edgeCosts.end()) {
            return it->second;
        }
        
        if (graph->hasEdge(from, to)) {
            return graph->getEdgeWeight(from, to);
        }
        
        return std::numeric_limits<double>::infinity();
    }
    
    void updateVertex(int nodeId) {
        std::cout << "[D*LITE] Updating vertex " << nodeId << std::endl;
        
        if (nodeId != goalNode) {
            double minRhs = std::numeric_limits<double>::infinity();
            std::vector<int> neighbors = graph->getNeighbors(nodeId);
            
            for (int neighbor : neighbors) {
                double cost = getEdgeCost(nodeId, neighbor) + nodes[neighbor].gValue;
                minRhs = std::min(minRhs, cost);
            }
            
            nodes[nodeId].rhsValue = minRhs;
        }
        
        // Remove from queue if present
        if (nodes[nodeId].inQueue) {
            nodes[nodeId].inQueue = false;
        }
        
        // Add to queue if inconsistent
        if (nodes[nodeId].gValue != nodes[nodeId].rhsValue) {
            auto key = calculateKey(nodeId);
            nodes[nodeId].fValue = key.first;
            nodes[nodeId].inQueue = true;
            priorityQueue.push(nodes[nodeId]);
            
            std::cout << "[D*LITE] Added inconsistent vertex " << nodeId 
                      << " to queue (g=" << nodes[nodeId].gValue 
                      << ", rhs=" << nodes[nodeId].rhsValue << ")" << std::endl;
        }
    }
    
    void computeShortestPath() {
        std::cout << "[D*LITE] Computing shortest path..." << std::endl;
        
        while (!priorityQueue.empty()) {
            // Get next node with valid queue status
            DStarNode current;
            bool foundValid = false;
            
            while (!priorityQueue.empty() && !foundValid) {
                current = priorityQueue.top();
                priorityQueue.pop();
                
                if (nodes[current.nodeId].inQueue) {
                    foundValid = true;
                    nodes[current.nodeId].inQueue = false;
                }
            }
            
            if (!foundValid) break;
            
            auto currentKey = calculateKey(current.nodeId);
            auto startKey = calculateKey(startNode);
            
            if (currentKey.first > startKey.first || 
                (currentKey.first == startKey.first && currentKey.second > startKey.second)) {
                break;
            }
            
            if (nodes[startNode].gValue <= nodes[startNode].rhsValue) {
                break;
            }
            
            std::cout << "[D*LITE] Processing vertex " << current.nodeId 
                      << " (key=" << currentKey.first << "," << currentKey.second << ")" << std::endl;
            
            if (nodes[current.nodeId].gValue > nodes[current.nodeId].rhsValue) {
                // Overconsistent vertex
                nodes[current.nodeId].gValue = nodes[current.nodeId].rhsValue;
                
                std::vector<int> neighbors = graph->getNeighbors(current.nodeId);
                for (int neighbor : neighbors) {
                    updateVertex(neighbor);
                }
            } else {
                // Underconsistent vertex
                nodes[current.nodeId].gValue = std::numeric_limits<double>::infinity();
                updateVertex(current.nodeId);
                
                std::vector<int> neighbors = graph->getNeighbors(current.nodeId);
                for (int neighbor : neighbors) {
                    updateVertex(neighbor);
                }
            }
        }
        
        std::cout << "[D*LITE] Shortest path computation completed" << std::endl;
    }
    
public:
    DStarLite(const Graph* environment) : graph(environment), keyModifier(0.0) {
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        std::cout << "[D*LITE] D* Lite algorithm initialized" << std::endl;
    }
    
    void initialize(int start, int goal) {
        std::cout << "[D*LITE] Initializing with start=" << start << ", goal=" << goal << std::endl;
        
        startNode = start;
        goalNode = goal;
        keyModifier = 0.0;
        
        // Initialize all nodes
        nodes.clear();
        while (!priorityQueue.empty()) priorityQueue.pop();
        
        for (int nodeId : graph->getAllNodeIds()) {
            nodes[nodeId] = DStarNode(nodeId);
        }
        
        // Initialize goal node
        nodes[goalNode].rhsValue = 0.0;
        auto goalKey = calculateKey(goalNode);
        nodes[goalNode].fValue = goalKey.first;
        nodes[goalNode].inQueue = true;
        priorityQueue.push(nodes[goalNode]);
        
        // Initialize edge costs
        edgeCosts.clear();
        for (int nodeId : graph->getAllNodeIds()) {
            std::vector<int> neighbors = graph->getNeighbors(nodeId);
            for (int neighbor : neighbors) {
                auto key = std::make_pair(nodeId, neighbor);
                edgeCosts[key] = graph->getEdgeWeight(nodeId, neighbor);
            }
        }
        
        std::cout << "[D*LITE] Initialization completed" << std::endl;
    }
    
    std::vector<int> findPath() {
        std::cout << "[D*LITE] Finding initial path from " << startNode << " to " << goalNode << std::endl;
        
        computeShortestPath();
        
        if (nodes[startNode].gValue == std::numeric_limits<double>::infinity()) {
            std::cout << "[D*LITE] No path found to goal" << std::endl;
            return {};
        }
        
        return reconstructPath();
    }
    
    std::vector<int> replan(int newStart, const std::vector<std::pair<std::pair<int, int>, double>>& edgeUpdates) {
        std::cout << "[D*LITE] Replanning from new start " << newStart << std::endl;
        
        // Update key modifier for new start position
        if (newStart != startNode) {
            keyModifier += calculateHeuristic(startNode, newStart);
            startNode = newStart;
        }
        
        // Process edge cost changes
        for (const auto& update : edgeUpdates) {
            int from = update.first.first;
            int to = update.first.second;
            double newCost = update.second;
            
            auto edgeKey = std::make_pair(from, to);
            double oldCost = getEdgeCost(from, to);
            
            if (oldCost != newCost) {
                std::cout << "[D*LITE] Edge cost changed: (" << from << "," << to 
                          << ") " << oldCost << " -> " << newCost << std::endl;
                
                edgeCosts[edgeKey] = newCost;
                updateVertex(from);
            }
        }
        
        computeShortestPath();
        
        if (nodes[startNode].gValue == std::numeric_limits<double>::infinity()) {
            std::cout << "[D*LITE] No path found after replanning" << std::endl;
            return {};
        }
        
        return reconstructPath();
    }
    
    std::vector<int> reconstructPath() const {
        std::cout << "[D*LITE] Reconstructing path..." << std::endl;
        
        std::vector<int> path;
        int current = startNode;
        path.push_back(current);
        
        while (current != goalNode) {
            std::vector<int> neighbors = graph->getNeighbors(current);
            int bestNext = -1;
            double bestCost = std::numeric_limits<double>::infinity();
            
            for (int neighbor : neighbors) {
                double cost = getEdgeCost(current, neighbor) + nodes.at(neighbor).gValue;
                if (cost < bestCost) {
                    bestCost = cost;
                    bestNext = neighbor;
                }
            }
            
            if (bestNext == -1) {
                std::cout << "[D*LITE] Path reconstruction failed - no valid next node" << std::endl;
                return {};
            }
            
            current = bestNext;
            path.push_back(current);
            
            if (path.size() > graph->getNodeCount()) {
                std::cout << "[D*LITE] Path reconstruction failed - cycle detected" << std::endl;
                return {};
            }
        }
        
        std::cout << "[D*LITE] Path reconstructed with " << path.size() << " nodes" << std::endl;
        return path;
    }
    
    void printStatistics() const {
        std::cout << "[D*LITE] Algorithm Statistics:" << std::endl;
        std::cout << "[D*LITE]   Start node: " << startNode << std::endl;
        std::cout << "[D*LITE]   Goal node: " << goalNode << std::endl;
        std::cout << "[D*LITE]   Key modifier: " << keyModifier << std::endl;
        std::cout << "[D*LITE]   Queue size: " << priorityQueue.size() << std::endl;
        std::cout << "[D*LITE]   Known edge costs: " << edgeCosts.size() << std::endl;
        
        if (nodes.find(startNode) != nodes.end()) {
            std::cout << "[D*LITE]   Start g-value: " << nodes.at(startNode).gValue << std::endl;
            std::cout << "[D*LITE]   Start rhs-value: " << nodes.at(startNode).rhsValue << std::endl;
        }
    }
};
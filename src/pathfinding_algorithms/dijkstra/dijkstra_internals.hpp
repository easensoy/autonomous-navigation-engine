#pragma once
// Private to the Dijkstra implementation (not installed). Defines the nested helper classes that
// Dijkstra.hpp only forward-declares, so every Dijkstra source file sees them complete and
// Dijkstra's out-of-line destructor can delete them.
#include "pathfinding_algorithms/Dijkstra.hpp"
#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Moved verbatim from priority_queue_ops.cpp.
class Dijkstra::PriorityQueueManager {
private:
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> pq;
    std::unordered_map<int, double> bestDistances;
    
public:
    void push(const DijkstraNode& node) {
        auto it = bestDistances.find(node.nodeId);
        if (it == bestDistances.end() || node.distance < it->second) {
            pq.push(node);
            bestDistances[node.nodeId] = node.distance;
        }
    }
    
    DijkstraNode pop() {
        while (!pq.empty()) {
            DijkstraNode current = pq.top();
            pq.pop();
            
            auto it = bestDistances.find(current.nodeId);
            if (it != bestDistances.end() && current.distance <= it->second) {
                return current;
            }
        }
        throw std::runtime_error("Priority queue is empty");
    }
    
    bool empty() const {
        return pq.empty();
    }
    
    size_t size() const {
        return pq.size();
    }
    
    void clear() {
        while (!pq.empty()) {
            pq.pop();
        }
        bestDistances.clear();
    }
    
    bool hasImprovedDistance(int nodeId, double distance) const {
        auto it = bestDistances.find(nodeId);
        return it == bestDistances.end() || distance < it->second;
    }
    
    void updateBestDistance(int nodeId, double distance) {
        bestDistances[nodeId] = distance;
    }
};

// Moved verbatim from shortest_path_tree.cpp.
class Dijkstra::ShortestPathTreeManager {
private:
    std::unordered_map<int, std::vector<int>> tree;
    std::unordered_map<int, double> nodeDistances;
    int rootNode;
    bool treeBuilt;
    
public:
    ShortestPathTreeManager() : rootNode(-1), treeBuilt(false) {}
    
    void buildTree(int startNode, const std::unordered_map<int, int>& predecessors, 
                   const std::unordered_map<int, double>& distances) {
        tree.clear();
        nodeDistances = distances;
        rootNode = startNode;
        treeBuilt = false;
        
        for (const auto& pair : predecessors) {
            int nodeId = pair.first;
            int predId = pair.second;
            
            if (predId != -1) {
                tree[predId].push_back(nodeId);
            }
        }
        
        treeBuilt = true;
        std::cout << "[DIJKSTRA] Built shortest path tree with root " << startNode << std::endl;
        std::cout << "[DIJKSTRA] Tree contains " << tree.size() << " internal nodes" << std::endl;
    }
    
    std::vector<std::vector<int>> getAllPaths(int startNode) const {
        if (!treeBuilt || startNode != rootNode) {
            return {};
        }
        
        std::vector<std::vector<int>> allPaths;
        std::vector<int> currentPath;
        currentPath.push_back(startNode);
        
        generateAllPaths(startNode, currentPath, allPaths);
        return allPaths;
    }
    
    std::vector<int> getPathTo(int targetNode, const std::unordered_map<int, int>& predecessors) const {
        std::vector<int> path;
        int current = targetNode;
        
        while (current != -1) {
            path.push_back(current);
            if (current == rootNode) break;
            
            auto it = predecessors.find(current);
            if (it != predecessors.end()) {
                current = it->second;
            } else {
                return {}; 
            }
        }
        
        if (path.empty() || path.back() != rootNode) {
            return {}; 
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }
    
    std::vector<int> getChildren(int nodeId) const {
        auto it = tree.find(nodeId);
        return (it != tree.end()) ? it->second : std::vector<int>();
    }
    
    bool isInTree(int nodeId) const {
        return nodeDistances.find(nodeId) != nodeDistances.end() && 
               nodeDistances.at(nodeId) != std::numeric_limits<double>::infinity();
    }
    
    double getDistanceInTree(int nodeId) const {
        auto it = nodeDistances.find(nodeId);
        return (it != nodeDistances.end()) ? it->second : std::numeric_limits<double>::infinity();
    }
    
    size_t getTreeSize() const {
        return nodeDistances.size();
    }
    
    int getTreeRoot() const {
        return rootNode;
    }
    
    void printTreeStructure() const {
        if (!treeBuilt) {
            std::cout << "[DIJKSTRA] Tree not built yet" << std::endl;
            return;
        }
        
        std::cout << "[DIJKSTRA] Shortest Path Tree Structure:" << std::endl;
        std::cout << "[DIJKSTRA] Root: " << rootNode << " (distance: 0)" << std::endl;
        
        printSubtree(rootNode, 0);
    }
    
private:
    void generateAllPaths(int currentNode, std::vector<int>& currentPath, 
                         std::vector<std::vector<int>>& allPaths) const {
        auto it = tree.find(currentNode);
        if (it == tree.end() || it->second.empty()) {
            // Leaf node - complete path
            allPaths.push_back(currentPath);
            return;
        }
        
        for (int child : it->second) {
            currentPath.push_back(child);
            generateAllPaths(child, currentPath, allPaths);
            currentPath.pop_back();
        }
    }
    
    void printSubtree(int nodeId, int depth) const {
        auto it = tree.find(nodeId);
        if (it != tree.end()) {
            for (int child : it->second) {
                for (int i = 0; i < depth + 1; ++i) {
                    std::cout << "  ";
                }
                std::cout << "└─ " << child << " (distance: " 
                          << getDistanceInTree(child) << ")" << std::endl;
                printSubtree(child, depth + 1);
            }
        }
    }
};

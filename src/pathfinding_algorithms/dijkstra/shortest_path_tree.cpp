#include "../../../include/pathfinding_algorithms/Dijkstra.hpp"
#include <iostream>
#include <algorithm>

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
                return {}; // No path exists
            }
        }
        
        if (path.empty() || path.back() != rootNode) {
            return {}; // Invalid path
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

void Dijkstra::buildShortestPathTree(int startNode) {
    if (!relaxationManager) {
        throw std::runtime_error("Cannot build tree without completed distance calculation");
    }
    
    if (!treeManager) {
        treeManager = std::make_unique<ShortestPathTreeManager>();
    }
    
    auto predecessors = relaxationManager->getAllPredecessors();
    auto distances = relaxationManager->getAllDistances();
    
    treeManager->buildTree(startNode, predecessors, distances);
}

std::vector<std::vector<int>> Dijkstra::findAllShortestPaths(int startId) {
    auto distances = findShortestDistances(startId);
    
    if (!treeManager) {
        buildShortestPathTree(startId);
    }
    
    return treeManager->getAllPaths(startId);
}

std::vector<int> Dijkstra::getShortestPathInTree(int targetNode) const {
    if (!treeManager || !relaxationManager) {
        return {};
    }
    
    auto predecessors = relaxationManager->getAllPredecessors();
    return treeManager->getPathTo(targetNode, predecessors);
}

std::vector<int> Dijkstra::getTreeChildren(int nodeId) const {
    return treeManager ? treeManager->getChildren(nodeId) : std::vector<int>();
}

bool Dijkstra::isNodeInTree(int nodeId) const {
    return treeManager && treeManager->isInTree(nodeId);
}

size_t Dijkstra::getShortestPathTreeSize() const {
    return treeManager ? treeManager->getTreeSize() : 0;
}

void Dijkstra::printShortestPathTree() const {
    if (treeManager) {
        treeManager->printTreeStructure();
    } else {
        std::cout << "[DIJKSTRA] No shortest path tree available" << std::endl;
    }
}

void Dijkstra::clearShortestPathTree() {
    if (treeManager) {
        treeManager.reset();
    }
}
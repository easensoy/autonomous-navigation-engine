#include "../../../include/pathfinding_algorithms/DFS.hpp"
#include <stack>
#include <iostream>
#include <algorithm>

DFS::DFS(const Graph* environment) : graph(environment) {
    if (!graph) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
    std::cout << "[DFS] Depth-First Search algorithm initialized" << std::endl;
}

std::vector<int> DFS::findPath(int startId, int goalId) {
    std::cout << "[DFS] Finding path from " << startId << " to " << goalId 
              << " using recursive approach" << std::endl;
    return findPathRecursive(startId, goalId);
}

std::vector<int> DFS::findPathIterative(int startId, int goalId) {
    std::cout << "[DFS] Finding path iteratively from " << startId << " to " << goalId << std::endl;
    
    if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
        throw std::invalid_argument("Start or goal node does not exist");
    }
    
    if (startId == goalId) {
        return {startId};
    }
    
    std::stack<std::pair<int, std::vector<int>>> searchStack;
    std::unordered_set<int> visited;
    
    searchStack.push({startId, {startId}});
    
    while (!searchStack.empty()) {
        auto [currentNode, currentPath] = searchStack.top();
        searchStack.pop();
        
        if (visited.find(currentNode) != visited.end()) {
            continue;
        }
        
        visited.insert(currentNode);
        
        std::cout << "[DFS] Exploring node " << currentNode 
                  << " at depth " << currentPath.size() << std::endl;
        
        if (currentNode == goalId) {
            std::cout << "[DFS] Goal found at depth " << currentPath.size() << std::endl;
            return currentPath;
        }
        
        std::vector<int> neighbors = graph->getNeighbors(currentNode);
        for (int neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end()) {
                std::vector<int> newPath = currentPath;
                newPath.push_back(neighbor);
                searchStack.push({neighbor, newPath});
            }
        }
    }
    
    std::cout << "[DFS] No path found to goal" << std::endl;
    return {};
}

std::vector<int> DFS::findPathRecursive(int startId, int goalId) {
    std::cout << "[DFS] Starting recursive search from " << startId << " to " << goalId << std::endl;
    
    if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
        throw std::invalid_argument("Start or goal node does not exist");
    }
    
    visited.clear();
    std::vector<int> path;
    
    if (dfsRecursive(startId, goalId, path)) {
        std::cout << "[DFS] Recursive search successful, path length: " << path.size() << std::endl;
        return path;
    }
    
    std::cout << "[DFS] Recursive search failed to find path" << std::endl;
    return {};
}

bool DFS::dfsRecursive(int currentId, int goalId, std::vector<int>& path) const {
    visited.insert(currentId);
    path.push_back(currentId);
    
    std::cout << "[DFS] Recursively visiting node " << currentId 
              << " at depth " << path.size() << std::endl;
    
    if (currentId == goalId) {
        std::cout << "[DFS] Goal reached in recursive search" << std::endl;
        return true;
    }
    
    std::vector<int> neighbors = graph->getNeighbors(currentId);
    for (int neighbor : neighbors) {
        if (visited.find(neighbor) == visited.end()) {
            if (dfsRecursive(neighbor, goalId, path)) {
                return true;
            }
        }
    }
    
    path.pop_back();
    std::cout << "[DFS] Backtracking from node " << currentId << std::endl;
    return false;
}

std::vector<int> DFS::getTraversalOrder(int startId) {
    std::cout << "[DFS] Computing traversal order from node " << startId << std::endl;
    
    if (!graph->hasNode(startId)) {
        throw std::invalid_argument("Start node does not exist");
    }
    
    std::vector<int> traversalOrder;
    visited.clear();
    dfsTraversal(startId, traversalOrder);
    
    std::cout << "[DFS] Traversal completed, visited " << traversalOrder.size() << " nodes" << std::endl;
    return traversalOrder;
}

void DFS::dfsTraversal(int startId, std::vector<int>& traversalOrder) const {
    visited.insert(startId);
    traversalOrder.push_back(startId);
    
    std::cout << "[DFS] Added node " << startId << " to traversal order" << std::endl;
    
    std::vector<int> neighbors = graph->getNeighbors(startId);
    for (int neighbor : neighbors) {
        if (visited.find(neighbor) == visited.end()) {
            dfsTraversal(neighbor, traversalOrder);
        }
    }
}

bool DFS::hasPath(int startId, int goalId) {
    return !findPath(startId, goalId).empty();
}

std::vector<std::vector<int>> DFS::findAllPaths(int startId, int goalId) {
    std::cout << "[DFS] Finding all paths from " << startId << " to " << goalId << std::endl;
    
    if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
        throw std::invalid_argument("Start or goal node does not exist");
    }
    
    std::vector<std::vector<int>> allPaths;
    std::vector<int> currentPath;
    std::unordered_set<int> pathVisited;
    
    findAllPathsRecursive(startId, goalId, currentPath, pathVisited, allPaths);
    
    std::cout << "[DFS] Found " << allPaths.size() << " total paths" << std::endl;
    return allPaths;
}

void DFS::findAllPathsRecursive(int current, int goal, std::vector<int>& currentPath,
                               std::unordered_set<int>& pathVisited, 
                               std::vector<std::vector<int>>& allPaths) const {
    currentPath.push_back(current);
    pathVisited.insert(current);
    
    if (current == goal) {
        allPaths.push_back(currentPath);
        std::cout << "[DFS] Found path of length " << currentPath.size() << std::endl;
    } else {
        std::vector<int> neighbors = graph->getNeighbors(current);
        for (int neighbor : neighbors) {
            if (pathVisited.find(neighbor) == pathVisited.end()) {
                findAllPathsRecursive(neighbor, goal, currentPath, pathVisited, allPaths);
            }
        }
    }
    
    currentPath.pop_back();
    pathVisited.erase(current);
}

void DFS::detectCycles(std::vector<std::vector<int>>& cycles) {
    std::cout << "[DFS] Detecting cycles in graph" << std::endl;
    
    cycles.clear();
    std::unordered_set<int> globalVisited;
    std::unordered_set<int> recursionStack;
    std::vector<int> currentPath;
    
    for (int nodeId : graph->getAllNodeIds()) {
        if (globalVisited.find(nodeId) == globalVisited.end()) {
            detectCyclesRecursive(nodeId, globalVisited, recursionStack, currentPath, cycles);
        }
    }
    
    std::cout << "[DFS] Cycle detection completed, found " << cycles.size() << " cycles" << std::endl;
}

void DFS::detectCyclesRecursive(int current, std::unordered_set<int>& globalVisited,
                               std::unordered_set<int>& recursionStack, 
                               std::vector<int>& currentPath,
                               std::vector<std::vector<int>>& cycles) const {
    globalVisited.insert(current);
    recursionStack.insert(current);
    currentPath.push_back(current);
    
    std::vector<int> neighbors = graph->getNeighbors(current);
    for (int neighbor : neighbors) {
        if (recursionStack.find(neighbor) != recursionStack.end()) {
            auto cycleStart = std::find(currentPath.begin(), currentPath.end(), neighbor);
            if (cycleStart != currentPath.end()) {
                std::vector<int> cycle(cycleStart, currentPath.end());
                cycle.push_back(neighbor);
                cycles.push_back(cycle);
                std::cout << "[DFS] Cycle detected with " << cycle.size() << " nodes" << std::endl;
            }
        } else if (globalVisited.find(neighbor) == globalVisited.end()) {
            detectCyclesRecursive(neighbor, globalVisited, recursionStack, currentPath, cycles);
        }
    }
    
    recursionStack.erase(current);
    currentPath.pop_back();
}

void DFS::printSearchProcess(bool enable) {
    searchLogging = enable;
    std::cout << "[DFS] Search process logging " << (enable ? "enabled" : "disabled") << std::endl;
}
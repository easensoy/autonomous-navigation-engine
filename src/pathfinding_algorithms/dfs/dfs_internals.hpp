#pragma once
// Private to the DFS implementation (not installed). Defines the nested helper classes that
// DFS.hpp only forward-declares, so every DFS source file sees them complete and
// DFS's out-of-line destructor can delete them.
#include "pathfinding_algorithms/DFS.hpp"
#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <stack>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <utility>

// Moved verbatim from stack_operations.cpp.
class DFS::StackManager {
private:
    std::stack<int> nodeStack;
    std::stack<std::vector<int>> pathStack;
    size_t maxStackSize;
    size_t totalPushed;
    size_t totalPopped;
    
public:
    StackManager() : maxStackSize(0), totalPushed(0), totalPopped(0) {}
    
    void pushNode(int nodeId) {
        nodeStack.push(nodeId);
        totalPushed++;
        
        if (nodeStack.size() > maxStackSize) {
            maxStackSize = nodeStack.size();
        }
    }
    
    void pushNodeWithPath(int nodeId, const std::vector<int>& path) {
        nodeStack.push(nodeId);
        pathStack.push(path);
        totalPushed++;
        
        if (nodeStack.size() > maxStackSize) {
            maxStackSize = nodeStack.size();
        }
    }
    
    int popNode() {
        if (nodeStack.empty()) {
            throw std::runtime_error("Cannot pop from empty stack");
        }
        
        int node = nodeStack.top();
        nodeStack.pop();
        totalPopped++;
        
        if (!pathStack.empty()) {
            pathStack.pop();
        }
        
        return node;
    }
    
    std::pair<int, std::vector<int>> popNodeWithPath() {
        if (nodeStack.empty() || pathStack.empty()) {
            throw std::runtime_error("Cannot pop from empty stack");
        }
        
        int node = nodeStack.top();
        std::vector<int> path = pathStack.top();
        
        nodeStack.pop();
        pathStack.pop();
        totalPopped++;
        
        return {node, path};
    }
    
    int top() const {
        if (nodeStack.empty()) {
            throw std::runtime_error("Stack is empty");
        }
        return nodeStack.top();
    }
    
    bool empty() const {
        return nodeStack.empty();
    }
    
    size_t size() const {
        return nodeStack.size();
    }
    
    void clear() {
        while (!nodeStack.empty()) {
            nodeStack.pop();
        }
        while (!pathStack.empty()) {
            pathStack.pop();
        }
    }
    
    size_t getMaxStackSize() const {
        return maxStackSize;
    }
    
    size_t getTotalPushed() const {
        return totalPushed;
    }
    
    size_t getTotalPopped() const {
        return totalPopped;
    }
    
    void resetStatistics() {
        maxStackSize = nodeStack.size();
        totalPushed = 0;
        totalPopped = 0;
    }
    
    void printStatistics() const {
        std::cout << "[DFS_STACK] Stack Statistics:" << std::endl;
        std::cout << "[DFS_STACK]   Current size: " << size() << std::endl;
        std::cout << "[DFS_STACK]   Maximum depth reached: " << maxStackSize << std::endl;
        std::cout << "[DFS_STACK]   Total pushed: " << totalPushed << std::endl;
        std::cout << "[DFS_STACK]   Total popped: " << totalPopped << std::endl;
        std::cout << "[DFS_STACK]   Current depth: " << (totalPushed - totalPopped) << std::endl;
    }
};

// Moved verbatim from recursive_search.cpp.
class DFS::RecursiveSearchManager {
private:
    mutable size_t recursionDepth;
    mutable size_t maxRecursionDepth;
    mutable size_t totalRecursiveCalls;
    mutable std::vector<int> recursionPath;
    bool recursionLogging;
    
public:
    RecursiveSearchManager() : recursionDepth(0), maxRecursionDepth(0), 
                              totalRecursiveCalls(0), recursionLogging(false) {}
    
    bool findPathRecursively(const Graph* graph, int current, int goal, 
                           std::unordered_set<int>& visited, std::vector<int>& path) const {
        recursionDepth++;
        totalRecursiveCalls++;
        
        if (recursionDepth > maxRecursionDepth) {
            maxRecursionDepth = recursionDepth;
        }
        
        visited.insert(current);
        path.push_back(current);
        recursionPath.push_back(current);
        
        if (recursionLogging) {
            std::cout << "[DFS_RECURSIVE] Depth " << recursionDepth 
                      << ": Visiting node " << current << std::endl;
        }
        
        if (current == goal) {
            if (recursionLogging) {
                std::cout << "[DFS_RECURSIVE] Goal found at depth " << recursionDepth << std::endl;
            }
            recursionDepth--;
            return true;
        }
        
        std::vector<int> neighbors = graph->getNeighbors(current);
        for (int neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end()) {
                if (findPathRecursively(graph, neighbor, goal, visited, path)) {
                    recursionDepth--;
                    return true;
                }
            }
        }
        
        path.pop_back();
        recursionPath.pop_back();
        
        if (recursionLogging) {
            std::cout << "[DFS_RECURSIVE] Depth " << recursionDepth 
                      << ": Backtracking from node " << current << std::endl;
        }
        
        recursionDepth--;
        return false;
    }
    
    void findAllPathsRecursively(const Graph* graph, int current, int goal,
                               std::vector<int>& currentPath, 
                               std::unordered_set<int>& pathVisited,
                               std::vector<std::vector<int>>& allPaths) const {
        recursionDepth++;
        totalRecursiveCalls++;
        
        if (recursionDepth > maxRecursionDepth) {
            maxRecursionDepth = recursionDepth;
        }
        
        currentPath.push_back(current);
        pathVisited.insert(current);
        
        if (recursionLogging) {
            std::cout << "[DFS_RECURSIVE] Exploring path of length " 
                      << currentPath.size() << " at node " << current << std::endl;
        }
        
        if (current == goal) {
            allPaths.push_back(currentPath);
            if (recursionLogging) {
                std::cout << "[DFS_RECURSIVE] Complete path found, length: " 
                          << currentPath.size() << std::endl;
            }
        } else {
            std::vector<int> neighbors = graph->getNeighbors(current);
            for (int neighbor : neighbors) {
                if (pathVisited.find(neighbor) == pathVisited.end()) {
                    findAllPathsRecursively(graph, neighbor, goal, currentPath, 
                                          pathVisited, allPaths);
                }
            }
        }
        
        currentPath.pop_back();
        pathVisited.erase(current);
        recursionDepth--;
    }
    
    void traverseRecursively(const Graph* graph, int current, 
                           std::unordered_set<int>& visited,
                           std::vector<int>& traversalOrder) const {
        recursionDepth++;
        totalRecursiveCalls++;
        
        if (recursionDepth > maxRecursionDepth) {
            maxRecursionDepth = recursionDepth;
        }
        
        visited.insert(current);
        traversalOrder.push_back(current);
        
        if (recursionLogging) {
            std::cout << "[DFS_RECURSIVE] Traversal depth " << recursionDepth 
                      << ": Added node " << current << std::endl;
        }
        
        std::vector<int> neighbors = graph->getNeighbors(current);
        for (int neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end()) {
                traverseRecursively(graph, neighbor, visited, traversalOrder);
            }
        }
        
        recursionDepth--;
    }
    
    void detectCyclesRecursively(const Graph* graph, int current,
                               std::unordered_set<int>& globalVisited,
                               std::unordered_set<int>& recursionStack,
                               std::vector<int>& currentPath,
                               std::vector<std::vector<int>>& cycles) const {
        recursionDepth++;
        totalRecursiveCalls++;
        
        if (recursionDepth > maxRecursionDepth) {
            maxRecursionDepth = recursionDepth;
        }
        
        globalVisited.insert(current);
        recursionStack.insert(current);
        currentPath.push_back(current);
        
        if (recursionLogging) {
            std::cout << "[DFS_RECURSIVE] Cycle detection depth " << recursionDepth 
                      << ": Exploring node " << current << std::endl;
        }
        
        std::vector<int> neighbors = graph->getNeighbors(current);
        for (int neighbor : neighbors) {
            if (recursionStack.find(neighbor) != recursionStack.end()) {
                auto cycleStart = std::find(currentPath.begin(), currentPath.end(), neighbor);
                if (cycleStart != currentPath.end()) {
                    std::vector<int> cycle(cycleStart, currentPath.end());
                    cycle.push_back(neighbor);
                    cycles.push_back(cycle);
                    
                    if (recursionLogging) {
                        std::cout << "[DFS_RECURSIVE] Cycle detected with " 
                                  << cycle.size() << " nodes" << std::endl;
                    }
                }
            } else if (globalVisited.find(neighbor) == globalVisited.end()) {
                detectCyclesRecursively(graph, neighbor, globalVisited, recursionStack, 
                                      currentPath, cycles);
            }
        }
        
        recursionStack.erase(current);
        currentPath.pop_back();
        recursionDepth--;
    }
    
    void resetStatistics() {
        recursionDepth = 0;
        maxRecursionDepth = 0;
        totalRecursiveCalls = 0;
        recursionPath.clear();
    }
    
    void setLogging(bool enabled) {
        recursionLogging = enabled;
    }
    
    size_t getMaxRecursionDepth() const {
        return maxRecursionDepth;
    }
    
    size_t getTotalRecursiveCalls() const {
        return totalRecursiveCalls;
    }
    
    const std::vector<int>& getCurrentRecursionPath() const {
        return recursionPath;
    }
    
    void printStatistics() const {
        std::cout << "[DFS_RECURSIVE] Recursion Statistics:" << std::endl;
        std::cout << "[DFS_RECURSIVE]   Maximum recursion depth: " << maxRecursionDepth << std::endl;
        std::cout << "[DFS_RECURSIVE]   Total recursive calls: " << totalRecursiveCalls << std::endl;
        std::cout << "[DFS_RECURSIVE]   Current recursion depth: " << recursionDepth << std::endl;
    }
};

#include "../../../include/pathfinding_algorithms/BFS.hpp"
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <algorithm>

BFS::BFS(const Graph* environment) : graph(environment) {
    if (!graph) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
    std::cout << "[BFS] Breadth-First Search algorithm initialized with " 
              << graph->getNodeCount() << " nodes" << std::endl;
}

std::vector<int> BFS::findPath(int startId, int goalId) {
    std::cout << "[BFS] Finding path from node " << startId << " to node " << goalId << std::endl;
    
    if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
        throw std::invalid_argument("Start or goal node does not exist in graph");
    }
    
    if (startId == goalId) {
        std::cout << "[BFS] Start and goal are the same node" << std::endl;
        return {startId};
    }
    
    std::queue<int> frontier;
    std::unordered_set<int> visited;
    std::unordered_map<int, int> parent;
    
    frontier.push(startId);
    visited.insert(startId);
    parent[startId] = -1;
    
    std::cout << "[BFS] Beginning breadth-first exploration from node " << startId << std::endl;
    
    int explorationLevel = 0;
    int nodesAtCurrentLevel = 1;
    int nodesAtNextLevel = 0;
    
    while (!frontier.empty()) {
        int current = frontier.front();
        frontier.pop();
        nodesAtCurrentLevel--;
        
        std::cout << "[BFS] Exploring node " << current 
                  << " at level " << explorationLevel << std::endl;
        
        if (current == goalId) {
            std::cout << "[BFS] Goal node reached at exploration level " 
                      << explorationLevel << std::endl;
            return reconstructPath(parent, startId, goalId);
        }
        
        std::vector<int> neighbors = graph->getNeighbors(current);
        std::cout << "[BFS] Node " << current << " has " << neighbors.size() << " neighbors" << std::endl;
        
        for (int neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                parent[neighbor] = current;
                frontier.push(neighbor);
                nodesAtNextLevel++;
                
                std::cout << "[BFS] Added node " << neighbor 
                          << " to frontier (parent: " << current << ")" << std::endl;
                
                if (neighbor == goalId) {
                    std::cout << "[BFS] Goal node " << goalId 
                              << " discovered at level " << (explorationLevel + 1) << std::endl;
                    return reconstructPath(parent, startId, goalId);
                }
            }
        }
        
        if (nodesAtCurrentLevel == 0) {
            explorationLevel++;
            nodesAtCurrentLevel = nodesAtNextLevel;
            nodesAtNextLevel = 0;
            
            if (nodesAtCurrentLevel > 0) {
                std::cout << "[BFS] Moving to exploration level " << explorationLevel 
                          << " with " << nodesAtCurrentLevel << " nodes" << std::endl;
            }
        }
    }
    
    std::cout << "[BFS] No path found to goal node " << goalId << std::endl;
    std::cout << "[BFS] Explored " << visited.size() << " nodes across " 
              << explorationLevel << " levels" << std::endl;
    
    return {};
}

std::vector<int> BFS::findShortestUnweightedPath(int startId, int goalId) {
    std::cout << "[BFS] Finding shortest unweighted path from " << startId 
              << " to " << goalId << std::endl;
    
    return findPath(startId, goalId);
}

std::vector<std::vector<int>> BFS::findAllPaths(int startId, int goalId, int maxDepth) {
    std::cout << "[BFS] Finding all paths from " << startId << " to " << goalId;
    if (maxDepth > 0) {
        std::cout << " with maximum depth " << maxDepth;
    }
    std::cout << std::endl;
    
    if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
        throw std::invalid_argument("Start or goal node does not exist");
    }
    
    std::vector<std::vector<int>> allPaths;
    std::queue<std::vector<int>> pathQueue;
    std::vector<int> initialPath = {startId};
    pathQueue.push(initialPath);
    
    while (!pathQueue.empty()) {
        std::vector<int> currentPath = pathQueue.front();
        pathQueue.pop();
        
        int currentNode = currentPath.back();
        
        if (maxDepth > 0 && static_cast<int>(currentPath.size()) > maxDepth) {
            continue;
        }
        
        if (currentNode == goalId) {
            allPaths.push_back(currentPath);
            std::cout << "[BFS] Found path of length " << currentPath.size() << std::endl;
            continue;
        }
        
        std::vector<int> neighbors = graph->getNeighbors(currentNode);
        for (int neighbor : neighbors) {
            if (std::find(currentPath.begin(), currentPath.end(), neighbor) == currentPath.end()) {
                std::vector<int> newPath = currentPath;
                newPath.push_back(neighbor);
                pathQueue.push(newPath);
            }
        }
    }
    
    std::cout << "[BFS] Found " << allPaths.size() << " total paths" << std::endl;
    return allPaths;
}

bool BFS::isReachable(int startId, int goalId) {
    if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
        return false;
    }
    
    if (startId == goalId) {
        return true;
    }
    
    std::queue<int> frontier;
    std::unordered_set<int> visited;
    
    frontier.push(startId);
    visited.insert(startId);
    
    while (!frontier.empty()) {
        int current = frontier.front();
        frontier.pop();
        
        if (current == goalId) {
            return true;
        }
        
        std::vector<int> neighbors = graph->getNeighbors(current);
        for (int neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                frontier.push(neighbor);
            }
        }
    }
    
    return false;
}

int BFS::getShortestPathLength(int startId, int goalId) {
    std::vector<int> path = findPath(startId, goalId);
    return path.empty() ? -1 : static_cast<int>(path.size() - 1);
}

std::vector<int> BFS::getNodesAtDistance(int startId, int distance) {
    std::cout << "[BFS] Finding all nodes at distance " << distance 
              << " from node " << startId << std::endl;
    
    if (!graph->hasNode(startId)) {
        throw std::invalid_argument("Start node does not exist");
    }
    
    if (distance < 0) {
        return {};
    }
    
    if (distance == 0) {
        return {startId};
    }
    
    std::vector<int> result;
    std::queue<std::pair<int, int>> frontier;
    std::unordered_set<int> visited;
    
    frontier.push({startId, 0});
    visited.insert(startId);
    
    while (!frontier.empty()) {
        auto [currentNode, currentDistance] = frontier.front();
        frontier.pop();
        
        if (currentDistance == distance) {
            result.push_back(currentNode);
            continue;
        }
        
        if (currentDistance < distance) {
            std::vector<int> neighbors = graph->getNeighbors(currentNode);
            for (int neighbor : neighbors) {
                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    frontier.push({neighbor, currentDistance + 1});
                }
            }
        }
    }
    
    std::cout << "[BFS] Found " << result.size() << " nodes at distance " << distance << std::endl;
    return result;
}

void BFS::printTraversalOrder(bool enable) {
    traversalLogging = enable;
    std::cout << "[BFS] Traversal order logging " 
              << (enable ? "enabled" : "disabled") << std::endl;
}

std::vector<int> BFS::reconstructPath(const std::unordered_map<int, int>& parent, 
                                     int startId, int goalId) const {
    std::vector<int> path;
    int current = goalId;
    
    std::cout << "[BFS] Reconstructing path from goal to start" << std::endl;
    
    while (current != -1) {
        path.push_back(current);
        if (current == startId) {
            break;
        }
        
        auto it = parent.find(current);
        if (it != parent.end()) {
            current = it->second;
        } else {
            std::cout << "[BFS] Path reconstruction failed - broken parent chain" << std::endl;
            return {};
        }
    }
    
    if (path.empty() || path.back() != startId) {
        std::cout << "[BFS] Invalid path reconstructed" << std::endl;
        return {};
    }
    
    std::reverse(path.begin(), path.end());
    
    std::cout << "[BFS] Successfully reconstructed path of length " << path.size() << std::endl;
    return path;
}
#pragma once
// Private to the BFS implementation (not installed). Defines the nested helper classes that
// BFS.hpp only forward-declares, so every BFS source file sees them complete and
// BFS's out-of-line destructor can delete them.
#include "pathfinding_algorithms/BFS.hpp"
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
#include <utility>

// Moved verbatim from queue_operations.cpp.
class BFS::QueueManager {
private:
    std::queue<int> nodeQueue;
    std::queue<int> distanceQueue;
    size_t maxQueueSize;
    size_t totalEnqueued;
    size_t totalDequeued;
    
public:
    QueueManager() : maxQueueSize(0), totalEnqueued(0), totalDequeued(0) {}
    
    void enqueue(int nodeId) {
        nodeQueue.push(nodeId);
        totalEnqueued++;
        
        if (nodeQueue.size() > maxQueueSize) {
            maxQueueSize = nodeQueue.size();
        }
    }
    
    void enqueueWithDistance(int nodeId, int distance) {
        nodeQueue.push(nodeId);
        distanceQueue.push(distance);
        totalEnqueued++;
        
        if (nodeQueue.size() > maxQueueSize) {
            maxQueueSize = nodeQueue.size();
        }
    }
    
    int dequeue() {
        if (nodeQueue.empty()) {
            throw std::runtime_error("Cannot dequeue from empty queue");
        }
        
        int node = nodeQueue.front();
        nodeQueue.pop();
        totalDequeued++;
        
        if (!distanceQueue.empty()) {
            distanceQueue.pop();
        }
        
        return node;
    }
    
    std::pair<int, int> dequeueWithDistance() {
        if (nodeQueue.empty() || distanceQueue.empty()) {
            throw std::runtime_error("Cannot dequeue from empty queue");
        }
        
        int node = nodeQueue.front();
        int distance = distanceQueue.front();
        
        nodeQueue.pop();
        distanceQueue.pop();
        totalDequeued++;
        
        return {node, distance};
    }
    
    int front() const {
        if (nodeQueue.empty()) {
            throw std::runtime_error("Queue is empty");
        }
        return nodeQueue.front();
    }
    
    bool empty() const {
        return nodeQueue.empty();
    }
    
    size_t size() const {
        return nodeQueue.size();
    }
    
    void clear() {
        while (!nodeQueue.empty()) {
            nodeQueue.pop();
        }
        while (!distanceQueue.empty()) {
            distanceQueue.pop();
        }
    }
    
    size_t getMaxQueueSize() const {
        return maxQueueSize;
    }
    
    size_t getTotalEnqueued() const {
        return totalEnqueued;
    }
    
    size_t getTotalDequeued() const {
        return totalDequeued;
    }
    
    void resetStatistics() {
        maxQueueSize = nodeQueue.size();
        totalEnqueued = 0;
        totalDequeued = 0;
    }
    
    void printStatistics() const {
        std::cout << "[BFS_QUEUE] Queue Statistics:" << std::endl;
        std::cout << "[BFS_QUEUE]   Current size: " << size() << std::endl;
        std::cout << "[BFS_QUEUE]   Maximum size reached: " << maxQueueSize << std::endl;
        std::cout << "[BFS_QUEUE]   Total enqueued: " << totalEnqueued << std::endl;
        std::cout << "[BFS_QUEUE]   Total dequeued: " << totalDequeued << std::endl;
        std::cout << "[BFS_QUEUE]   Remaining in queue: " << (totalEnqueued - totalDequeued) << std::endl;
    }
};

// Moved verbatim from unweighted_pathfinding.cpp.
class BFS::UnweightedPathfinder {
private:
    const Graph* graph;
    bool optimizationEnabled;
    
public:
    explicit UnweightedPathfinder(const Graph* environment) 
        : graph(environment), optimizationEnabled(true) {}
    
    std::vector<int> findOptimalUnweightedPath(int startId, int goalId) {
        std::cout << "[BFS_UNWEIGHTED] Finding optimal unweighted path from " 
                  << startId << " to " << goalId << std::endl;
        
        if (startId == goalId) {
            return {startId};
        }
        
        std::queue<int> frontier;
        std::unordered_map<int, int> parent;
        std::unordered_map<int, int> distance;
        
        frontier.push(startId);
        parent[startId] = -1;
        distance[startId] = 0;
        
        while (!frontier.empty()) {
            int current = frontier.front();
            frontier.pop();
            
            if (current == goalId) {
                std::cout << "[BFS_UNWEIGHTED] Goal reached at distance " 
                          << distance[goalId] << std::endl;
                return reconstructOptimalPath(parent, startId, goalId);
            }
            
            std::vector<int> neighbors = graph->getNeighbors(current);
            for (int neighbor : neighbors) {
                if (distance.find(neighbor) == distance.end()) {
                    distance[neighbor] = distance[current] + 1;
                    parent[neighbor] = current;
                    frontier.push(neighbor);
                }
            }
        }
        
        std::cout << "[BFS_UNWEIGHTED] No path found" << std::endl;
        return {};
    }
    
    std::unordered_map<int, int> computeAllDistances(int startId) {
        std::cout << "[BFS_UNWEIGHTED] Computing distances from " << startId 
                  << " to all reachable nodes" << std::endl;
        
        std::unordered_map<int, int> distances;
        std::queue<int> frontier;
        
        frontier.push(startId);
        distances[startId] = 0;
        
        while (!frontier.empty()) {
            int current = frontier.front();
            frontier.pop();
            
            std::vector<int> neighbors = graph->getNeighbors(current);
            for (int neighbor : neighbors) {
                if (distances.find(neighbor) == distances.end()) {
                    distances[neighbor] = distances[current] + 1;
                    frontier.push(neighbor);
                }
            }
        }
        
        std::cout << "[BFS_UNWEIGHTED] Computed distances to " << distances.size() 
                  << " reachable nodes" << std::endl;
        
        return distances;
    }
    
    std::vector<std::vector<int>> findAllShortestPaths(int startId, int goalId) {
        std::cout << "[BFS_UNWEIGHTED] Finding all shortest paths from " 
                  << startId << " to " << goalId << std::endl;
        
        if (startId == goalId) {
            return {{startId}};
        }
        
        std::queue<int> frontier;
        std::unordered_map<int, int> distance;
        std::unordered_map<int, std::vector<int>> parents;
        
        frontier.push(startId);
        distance[startId] = 0;
        parents[startId] = {};
        
        while (!frontier.empty()) {
            int current = frontier.front();
            frontier.pop();
            
            if (current == goalId) {
                break;
            }
            
            std::vector<int> neighbors = graph->getNeighbors(current);
            for (int neighbor : neighbors) {
                int newDistance = distance[current] + 1;
                
                if (distance.find(neighbor) == distance.end()) {
                    distance[neighbor] = newDistance;
                    parents[neighbor] = {current};
                    frontier.push(neighbor);
                } else if (distance[neighbor] == newDistance) {
                    parents[neighbor].push_back(current);
                }
            }
        }
        
        if (distance.find(goalId) == distance.end()) {
            std::cout << "[BFS_UNWEIGHTED] No path exists to goal" << std::endl;
            return {};
        }
        
        std::vector<std::vector<int>> allPaths;
        std::vector<int> currentPath;
        generateAllShortestPaths(goalId, startId, parents, currentPath, allPaths);
        
        std::cout << "[BFS_UNWEIGHTED] Found " << allPaths.size() 
                  << " shortest paths of length " << distance[goalId] << std::endl;
        
        return allPaths;
    }
    
    std::vector<int> getNodesAtExactDistance(int startId, int targetDistance) {
        if (targetDistance < 0) {
            return {};
        }
        
        if (targetDistance == 0) {
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
            
            if (currentDistance == targetDistance) {
                result.push_back(currentNode);
                continue;
            }
            
            if (currentDistance < targetDistance) {
                std::vector<int> neighbors = graph->getNeighbors(currentNode);
                for (int neighbor : neighbors) {
                    if (visited.find(neighbor) == visited.end()) {
                        visited.insert(neighbor);
                        frontier.push({neighbor, currentDistance + 1});
                    }
                }
            }
        }
        
        return result;
    }
    
    bool hasPathWithMaxLength(int startId, int goalId, int maxLength) {
        if (maxLength < 0) {
            return false;
        }
        
        if (startId == goalId) {
            return maxLength >= 0;
        }
        
        std::queue<std::pair<int, int>> frontier;
        std::unordered_set<int> visited;
        
        frontier.push({startId, 0});
        visited.insert(startId);
        
        while (!frontier.empty()) {
            auto [currentNode, currentLength] = frontier.front();
            frontier.pop();
            
            if (currentNode == goalId) {
                return true;
            }
            
            if (currentLength < maxLength) {
                std::vector<int> neighbors = graph->getNeighbors(currentNode);
                for (int neighbor : neighbors) {
                    if (visited.find(neighbor) == visited.end()) {
                        visited.insert(neighbor);
                        frontier.push({neighbor, currentLength + 1});
                    }
                }
            }
        }
        
        return false;
    }
    
private:
    std::vector<int> reconstructOptimalPath(const std::unordered_map<int, int>& parent, 
                                          int startId, int goalId) {
        std::vector<int> path;
        int current = goalId;
        
        while (current != -1) {
            path.push_back(current);
            if (current == startId) {
                break;
            }
            current = parent.at(current);
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }
    
    void generateAllShortestPaths(int current, int start, 
                                const std::unordered_map<int, std::vector<int>>& parents,
                                std::vector<int>& currentPath, 
                                std::vector<std::vector<int>>& allPaths) {
        currentPath.push_back(current);
        
        if (current == start) {
            std::vector<int> path = currentPath;
            std::reverse(path.begin(), path.end());
            allPaths.push_back(path);
        } else {
            auto it = parents.find(current);
            if (it != parents.end()) {
                for (int parent : it->second) {
                    generateAllShortestPaths(parent, start, parents, currentPath, allPaths);
                }
            }
        }
        
        currentPath.pop_back();
    }
};

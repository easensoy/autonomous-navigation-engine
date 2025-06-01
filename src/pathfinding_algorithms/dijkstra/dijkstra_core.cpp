#include "../../../include/pathfinding_algorithms/Dijkstra.hpp"
#include <queue>
#include <unordered_map>
#include <iostream>
#include <limits>
#include <unordered_set>

Dijkstra::Dijkstra(const Graph* environment) : graph(environment) {}

Dijkstra::DijkstraNode::DijkstraNode(int id, double dist, int prev)
    : nodeId(id), distance(dist), previous(prev) {}

bool Dijkstra::DijkstraNode::operator>(const DijkstraNode& other) const {
    return distance > other.distance;
}

std::vector<int> Dijkstra::findShortestPath(int startId, int goalId) {
    std::cout << "Dijkstra's Algorithm: Finding shortest path from " << startId << " to " << goalId << std::endl;
    
    std::unordered_map<int, double> distances;
    std::unordered_map<int, int> previousNodes;
    std::unordered_set<int> visited;
    
    for (int nodeId : graph->getAllNodeIds()) {
        distances[nodeId] = std::numeric_limits<double>::infinity();
        visited[nodeId] = false;
    }
    
    distances[startId] = 0.0;
    
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> pq;
    pq.emplace(startId, 0.0, -1);
    
    while (!pq.empty()) {
        DijkstraNode current = pq.top();
        pq.pop();
        
        if (visited[current.nodeId]) {
            continue;
        }
        
        visited[current.nodeId] = true;
        
        if (current.nodeId == goalId) {
            std::cout << "Dijkstra's Algorithm: Goal reached!" << std::endl;
            return reconstructPath(previousNodes, startId, goalId);
        }
        
        for (const Edge& edge : graph->getEdgesFrom(current.nodeId)) {
            int neighborId = edge.getToNode();
            double newDistance = current.distance + edge.getWeight();
            
            if (newDistance < distances[neighborId]) {
                distances[neighborId] = newDistance;
                previousNodes[neighborId] = current.nodeId;
                pq.emplace(neighborId, newDistance);
            }
        }
    }
    
    std::cout << "Dijkstra's Algorithm: No path found" << std::endl;
    return {};
}

std::vector<int> Dijkstra::reconstructPath(const std::unordered_map<int, int>& previousNodes, int startId, int goalId) const {
    std::vector<int> path;
    int current = goalId;
    
    while (current != -1) {
        path.push_back(current);
        if (current == startId) break;
        
        auto it = previousNodes.find(current);
        if (it != previousNodes.end()) {
            current = it->second;
        } else {
            return {};
        }
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

std::unordered_map<int, double> Dijkstra::findShortestDistances(int startId) {
    std::unordered_map<int, double> distances;
    std::unordered_set<int> visited;
    
    for (int nodeId : graph->getAllNodeIds()) {
        distances[nodeId] = std::numeric_limits<double>::infinity();
    }
    
    distances[startId] = 0.0;
    
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> pq;
    pq.emplace(startId, 0.0);
    
    while (!pq.empty()) {
        DijkstraNode current = pq.top();
        pq.pop();
        
        if (visited.find(current.nodeId) != visited.end()) {
            continue;
        }
        
        visited.insert(current.nodeId);
        
        for (const Edge& edge : graph->getEdgesFrom(current.nodeId)) {
            int neighborId = edge.getToNode();
            double newDistance = current.distance + edge.getWeight();
            
            if (newDistance < distances[neighborId]) {
                distances[neighborId] = newDistance;
                pq.emplace(neighborId, newDistance);
            }
        }
    }
    
    return distances;
}

double Dijkstra::getShortestDistance(int startId, int goalId) {
    auto distances = findShortestDistances(startId);
    return distances[goalId];
}

bool Dijkstra::hasPath(int startId, int goalId) {
    return getShortestDistance(startId, goalId) != std::numeric_limits<double>::infinity();
}

void Dijkstra::printAlgorithmSteps(bool enable) {
    std::cout << "Dijkstra step printing " << (enable ? "enabled" : "disabled") << std::endl;
}
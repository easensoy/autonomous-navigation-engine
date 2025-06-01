#pragma once
#include "../core/Graph.hpp"
#include <vector>
#include <unordered_map>

class Dijkstra
{
    private:
        const Graph* graph;

        struct DijkstraNode
        {
            int nodeId;
            double distance;
            int previous;

            DijkstraNode(int id, double dist, int prev = -1);
            bool operator>(const DijkstraNode& other) const;
        };

        std::vector<int> reconstructPath(const std::unordered_map<int, int>& previousNodes, int startId, int goalId) const;
    
    public:
        explicit Dijkstra(const Graph& environment);

        Dijkstra(const Graph *environment);

        std::vector<int> findShortestPath(int startId, int goalId);
        std::unordered_map<int, double> findShortestDistances(int startId);
        std::vector<std::vector<int>> findAllShortestPaths(int startId);

        double getShortestDistance(int startId, int goalId);
        bool hasPath(int startId, int goalId);
        void printAlgorithmSteps(bool enable);
};
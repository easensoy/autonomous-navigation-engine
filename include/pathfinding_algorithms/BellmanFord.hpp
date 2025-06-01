#pragma once
#include "../core/Graph.hpp"
#include <vector>
#include <unordered_map>
#include <limits>

class BellmanFord
{
    private:
        const Graph* graph;

        struct BellmanFordResult
        {
            std::unordered_map<int, double> distances;
            std::unordered_map<int, int> predecessors;
            bool hasNegativeCycle;
            std::vector<int> negativeCycle;
        };

        std::vector<int> reconstructPath(const std::unordered_map<int, int>& predecessors, int startId, int goalId) const;
        std::vector<int> findNegativeCycle(const std::unordered_map<int, int>& predecessors) const;
    
    public:
        explicit BellmanFord(const Graph* environment);

        std::vector<int> findShortestPath(int startId, int goalId);
        BellmanFordResult computeShortestPaths(int startId);
        bool detectNegativeCycle();
        std::vector<int> getNegativeCycle();

        double getShortestDistance(int startId, int goalId);
        bool canHandleNegativeWeights() const;
        void printIterationDetails(bool enable);
};
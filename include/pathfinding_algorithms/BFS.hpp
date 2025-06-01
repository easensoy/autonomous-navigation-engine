#pragma once
#include "../core/Graph.hpp"
#include <vector>
#include <queue>
#include <unordered_set>

class BFS
{
    private:
        const Graph* graph;

        std::vector<int> reconstructPath(const std::unordered_map<int, int>& parent, int startId, int goalId) const;
    
    public:
        explicit BFS(const Graph* environment);
        
        std::vector<int> findPath(int startId, int goalId);
        std::vector<int> findShortestUnweightedPath(int startId, int goalId);
        std::vector<std::vector<int>> findAllPaths(int startId, int goalId, int maxDepth = -1);

        bool isReachable(int startId, int goalId);
        int getShortestPathLength(int startId, int goalId);
        std::vector<int> getNodesAtDistance(int startId, int distance);

        void printTraversalOrder(bool enable);
};
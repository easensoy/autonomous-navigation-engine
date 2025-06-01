#pragma once
#include "../core/Graph.hpp"
#include <vector>
#include <stack>
#include <unordered_set>

class DFS
{
    private:
        const Graph* graph;
        mutable std::unordered_set<int> visited;

        bool dfsRecursive(int currentId, int goalId, std::vector<int>& path) const;
        void dfsTraversal(int startId, std::vector<int>& traversalOrder) const;

    public:
        explicit DFS(const Graph* environment);

        std::vector<int> findPath(int startId, int goalId);
        std::vector<int> findPathIterative(int startId, int goalId);
        std::vector<int> findPathRecursive(int startId, int goalId);

        std::vector<int> getTraversalOrder(int startId);
        bool hasPath(int startId, int goalId);
        std::vector<std::vector<int>> findAllPaths(int startId, int goalId);

        void detectCycles(std::vector<std::vector<int>>& cycles);
        void printSearchProcess(bool enable);
};
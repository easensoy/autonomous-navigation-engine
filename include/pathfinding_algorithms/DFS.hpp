#pragma once
#include "core/Graph.hpp"
#include <vector>
#include <stack>
#include <unordered_set>
#include <unordered_map>
#include <memory>

class DFS
{
    private:
        const Graph* graph;
        
        class StackManager;
        class RecursiveSearchManager;
        
        std::unique_ptr<StackManager> stackManager;
        std::unique_ptr<RecursiveSearchManager> recursiveManager;
        
        mutable std::unordered_set<int> visited;
        bool searchLogging;

        bool dfsRecursive(int currentId, int goalId, std::vector<int>& path) const;
        void dfsTraversal(int startId, std::vector<int>& traversalOrder) const;
        void findAllPathsRecursive(int current, int goal, std::vector<int>& currentPath,
                                 std::unordered_set<int>& pathVisited, 
                                 std::vector<std::vector<int>>& allPaths) const;
        void detectCyclesRecursive(int current, std::unordered_set<int>& globalVisited,
                                 std::unordered_set<int>& recursionStack, 
                                 std::vector<int>& currentPath,
                                 std::vector<std::vector<int>>& cycles) const;
        
        void initializeStack();
        void pushToStack(int nodeId);
        void pushToStackWithPath(int nodeId, const std::vector<int>& path);
        int popFromStack();
        std::pair<int, std::vector<int>> popFromStackWithPath();
        bool isStackEmpty() const;
        size_t getStackSize() const;
        size_t getMaxStackDepth() const;
        void clearStack();
        void printStackStatistics() const;
        
        void initializeRecursiveSearch();
        size_t getMaxRecursionDepth() const;
        size_t getTotalRecursiveCalls() const;
        void resetRecursiveStatistics();
        void setRecursiveLogging(bool enabled);
        void printRecursiveStatistics() const;

    public:
        explicit DFS(const Graph* environment);
        
        ~DFS() = default;
        
        DFS(const DFS& other) = delete;
        DFS& operator=(const DFS& other) = delete;
        DFS(DFS&& other) noexcept = default;
        DFS& operator=(DFS&& other) noexcept = default;

        std::vector<int> findPath(int startId, int goalId);
        std::vector<int> findPathIterative(int startId, int goalId);
        std::vector<int> findPathRecursive(int startId, int goalId);

        std::vector<int> getTraversalOrder(int startId);
        bool hasPath(int startId, int goalId);
        std::vector<std::vector<int>> findAllPaths(int startId, int goalId);

        void detectCycles(std::vector<std::vector<int>>& cycles);
        
        void printSearchProcess(bool enable);
};

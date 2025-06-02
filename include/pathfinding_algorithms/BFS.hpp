#pragma once
#include "../core/Graph.hpp"
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <memory>

class BFS
{
    private:
        const Graph* graph;
        
        class QueueManager;
        class UnweightedPathfinder;
        
        std::unique_ptr<QueueManager> queueManager;
        std::unique_ptr<UnweightedPathfinder> unweightedPathfinder;
        
        bool traversalLogging;

        std::vector<int> reconstructPath(const std::unordered_map<int, int>& parent, int startId, int goalId) const;
        void findAllPathsRecursive(int current, int goal, std::vector<int>& currentPath,
                                 std::unordered_set<int>& pathVisited, 
                                 std::vector<std::vector<int>>& allPaths) const;
        
        void initializeQueue();
        void enqueueNode(int nodeId);
        void enqueueNodeWithDistance(int nodeId, int distance);
        int dequeueNode();
        std::pair<int, int> dequeueNodeWithDistance();
        bool isQueueEmpty() const;
        size_t getQueueSize() const;
        size_t getMaxQueueSize() const;
        void clearQueue();
        void printQueueStatistics() const;
        
        std::vector<int> findOptimalUnweightedPath(int startId, int goalId);
        std::unordered_map<int, int> computeAllDistances(int startId);
        std::vector<std::vector<int>> findAllShortestPaths(int startId, int goalId);
        std::vector<int> getNodesAtExactDistance(int startId, int targetDistance);
        bool hasPathWithMaxLength(int startId, int goalId, int maxLength);
    
    public:
        explicit BFS(const Graph* environment);
        
        ~BFS() = default;
        
        BFS(const BFS& other) = delete;
        BFS& operator=(const BFS& other) = delete;
        BFS(BFS&& other) noexcept = default;
        BFS& operator=(BFS&& other) noexcept = default;
        
        std::vector<int> findPath(int startId, int goalId);
        std::vector<int> findShortestUnweightedPath(int startId, int goalId);
        std::vector<std::vector<int>> findAllPaths(int startId, int goalId, int maxDepth = -1);

        bool isReachable(int startId, int goalId);
        int getShortestPathLength(int startId, int goalId);
        std::vector<int> getNodesAtDistance(int startId, int distance);

        void printTraversalOrder(bool enable);
};

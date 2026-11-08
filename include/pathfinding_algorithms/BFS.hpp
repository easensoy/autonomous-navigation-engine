#pragma once
#include "core/Graph.hpp"
#include "pathfinding_algorithms/SearchObserver.hpp"
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <utility>

class BFS
{
    private:
        const Graph* graph;
        
        class QueueManager;
        class UnweightedPathfinder;
        
        std::unique_ptr<QueueManager> queueManager;
        std::unique_ptr<UnweightedPathfinder> unweightedPathfinder;
        
        bool traversalLogging;
        SearchObserver* searchObserver = nullptr;  // optional, read-only; see setSearchObserver()

        std::vector<int> reconstructPath(const std::unordered_map<int, int>& parent, int startId, int goalId) const;
        
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
        
    
    public:
        explicit BFS(const Graph* environment);
        
        ~BFS();  // defined in the .cpp, where the helper classes are complete
        
        BFS(const BFS& other) = delete;
        BFS& operator=(const BFS& other) = delete;
        BFS(BFS&& other) noexcept;
        BFS& operator=(BFS&& other) noexcept;
        
        std::vector<int> findPath(int startId, int goalId);
        std::vector<int> findShortestUnweightedPath(int startId, int goalId);
        std::vector<std::vector<int>> findAllPaths(int startId, int goalId, int maxDepth = -1);

        bool isReachable(int startId, int goalId);
        int getShortestPathLength(int startId, int goalId);
        std::vector<int> getNodesAtDistance(int startId, int distance);

        void printTraversalOrder(bool enable);

        // Attaches a read-only observer that findPath reports its dequeues and enqueues to, in
        // execution order; nullptr (the default) detaches it.
        void setSearchObserver(SearchObserver* observer);
};

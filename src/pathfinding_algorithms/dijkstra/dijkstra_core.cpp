#pragma once
#include "core/Graph.hpp"
#include <vector>
#include <unordered_map>
#include <memory>

class Dijkstra
{
    private:
        const Graph* graph;
        
        class DistanceRelaxationManager;
        class PriorityQueueManager;
        class ShortestPathTreeManager;
        
        std::unique_ptr<DistanceRelaxationManager> relaxationManager;
        std::unique_ptr<PriorityQueueManager> pqManager;
        std::unique_ptr<ShortestPathTreeManager> treeManager;

        struct DijkstraNode
        {
            int nodeId;
            double distance;
            int previous;

            DijkstraNode(int id, double dist, int prev = -1) 
                : nodeId(id), distance(dist), previous(prev) {}
            
            bool operator>(const DijkstraNode& other) const {
                return distance > other.distance;
            }
            
            bool operator<(const DijkstraNode& other) const {
                return distance < other.distance;
            }
        };

        std::vector<int> reconstructPath(const std::unordered_map<int, int>& previousNodes, int startId, int goalId) const;
        
        void initializeDistanceRelaxation(const std::vector<int>& nodeIds, int startId);
        bool relaxEdge(int fromNode, int toNode, double edgeWeight);
        double getCurrentDistance(int nodeId) const;
        int getPredecessor(int nodeId) const;
        std::unordered_map<int, double> getAllCurrentDistances() const;
        std::unordered_map<int, int> getAllPredecessors() const;
        bool isNodeReachable(int nodeId) const;
        size_t getReachableNodeCount() const;
        void setDistanceLogging(bool enabled);
        
        void initializePriorityQueue();
        void addToPriorityQueue(const DijkstraNode& node);
        DijkstraNode getNextFromPriorityQueue();
        bool isPriorityQueueEmpty() const;
        size_t getPriorityQueueSize() const;
        void clearPriorityQueue();
        bool shouldUpdateDistance(int nodeId, double newDistance) const;
        
        void buildShortestPathTree(int startNode);
        std::vector<int> getShortestPathInTree(int targetNode) const;
        std::vector<int> getTreeChildren(int nodeId) const;
        bool isNodeInTree(int nodeId) const;
        size_t getShortestPathTreeSize() const;
        void printShortestPathTree() const;
        void clearShortestPathTree();
    
    public:
        explicit Dijkstra(const Graph& environment);
        explicit Dijkstra(const Graph *environment);
        
        ~Dijkstra() = default;
        
        Dijkstra(const Dijkstra& other) = delete;
        Dijkstra& operator=(const Dijkstra& other) = delete;
        Dijkstra(Dijkstra&& other) noexcept = default;
        Dijkstra& operator=(Dijkstra&& other) noexcept = default;

        std::vector<int> findShortestPath(int startId, int goalId);
        std::unordered_map<int, double> findShortestDistances(int startId);
        std::vector<std::vector<int>> findAllShortestPaths(int startId);

        double getShortestDistance(int startId, int goalId);
        bool hasPath(int startId, int goalId);
        void printAlgorithmSteps(bool enable);
};
#pragma once
#include "../core/Graph.hpp"
#include <vector>
#include <unordered_map>
#include <limits>
#include <memory>
#include <chrono>
#include <iomanip>

class BellmanFord
{
    private:
        const Graph* graph;

        class NegativeCycleDetector;
        class EdgeRelaxationManager;

        std::unique_ptr<NegativeCycleDetector> cycleDetector;
        std::unique_ptr<EdgeRelaxationManager> relaxationManager;

        bool iterationLogging;

        struct BellmanFordResult
        {
            std::unordered_map<int, double> distances;
            std::unordered_map<int, int> predecessors;
            bool hasNegativeCycle;
            std::vector<int> negativeCycle;
        };

        std::vector<int> reconstructPath(const std::unordered_map<int, int>& predecessors, int startId, int goalId) const;
        std::vector<int> findNegativeCycle(const std::unordered_map<int, int>& predecessors) const;

        bool initializeNegativeCycleDetection();
        bool hasNegativeCycleInDistances(const std::unordered_map<int, double>& distances,
                                         const std::unordered_map<int, int>& predecessors);
        std::vector<int> getLastDetectedNegativeCycle() const;
        std::vector<int> findAllNegativeCycles();
        bool isNodeInNegativeCycle(int nodeId);
        void enableNegativeCycleDetection(bool enabled);
        double calculateNegativeCycleWeight(const std::vector<int>& cycle) const;

        void initializeEdgeRelaxation(const std::vector<int>& nodeIds, int startNode);
        bool relaxSingleEdge(int fromNode, int toNode, double edgeWeight);
        size_t performFullEdgeRelaxationPass();
        std::unordered_map<int, double> getCurrentDistancesFromRelaxation() const;
        std::unordered_map<int, int> getCurrentPredecessorsFromRelaxation() const;
        size_t getEdgeRelaxationCount() const;
        size_t getTotalEdgesProcessed() const;
        double getEdgeRelaxationRatio() const;
        void setEdgeRelaxationLogging(bool enabled);
        void resetEdgeRelaxationStatistics();
        std::vector<std::pair<int, double>> getReachableNodesWithDistances() const;
        void printEdgeRelaxationStatistics() const;

    public:
        explicit BellmanFord(const Graph* environment);

        ~BellmanFord() = default;

        BellmanFord(const BellmanFord& other) = delete;
        BellmanFord& operator=(const BellmanFord& other) = delete;
        BellmanFord(BellmanFord&& other) noexcept = default;
        BellmanFord& operator=(BellmanFord&& other) noexcept = default;

        std::vector<int> findShortestPath(int startId, int goalId);
        BellmanFordResult computeShortestPaths(int startId);
        double getShortestDistance(int startId, int goalId);

        bool detectNegativeCycle();
        std::vector<int> getNegativeCycle();

        bool canHandleNegativeWeights() const;
        void printIterationDetails(bool enable);
};

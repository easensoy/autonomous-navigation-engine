#pragma once
#include "core/Graph.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include <vector>
#include <memory>

enum class PlanningAlgorithm
{
    DIJKSTRA,
    ASTAR,
    BELLMAN_FORD,
    AUTO_SELECT
};

class GlobalPathPlanner
{
    private:
        const Graph* environment;
        PlanningAlgorithm currentAlgorithm;
        std::unique_ptr<AStar> aStarPlanner;
        std::unique_ptr<Dijkstra> dijkstraPlanner;

        PlanningAlgorithm selectOptimalAlgorithm(int startId, int goalId) const;

    public:
        explicit GlobalPathPlanner(const Graph* graph);
        ~GlobalPathPlanner() = default;

        std::vector<int> planPath(int startId, int goalId);
        std::vector<int> planPathWithAlgorithm(int startId, int goalId, PlanningAlgorithm algorithm);
        std::vector<std::vector<int>> planMultipleGoals(int startId, const std::vector<int>& goals);

        void setAlgorithm(PlanningAlgorithm algorithm);
        PlanningAlgorithm getCurrentAlgorithm() const;
        double estimatePathCost(int startId, int goalId) const;

        void optimizeForSpeed();
        void optimizeForAccuracy();
        void enableAutoSelection(bool enable);
};

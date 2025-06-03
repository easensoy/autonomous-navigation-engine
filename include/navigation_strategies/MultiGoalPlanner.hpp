#pragma once
#include "core/Graph.hpp"
#include "GlobalPathPlanner.hpp"
#include <vector>
#include <utility>

enum class MultiGoalStrategy
{
    NEAREST_NEIGHBOR,
    GENETIC_ALGORITHM,
    SIMULATED_ANNEALING,
    BRANCH_AND_BOUND
};

class MultiGoalPlanner
{
    private:
        const Graph* environment;
        std::unique_ptr<GlobalPathPlanner> globalPathPlanner;
        MultiGoalStrategy optimizationStrategy;

        std::vector<int> nearestNeighborTSP(int startId, std::vector<int> goals);
        std::vector<int> geneticAlgorithmTSP(int startId, std::vector<int> goals);
        std::vector<int> simulatedAnnealingTSP(int startId, std::vector<int> goals);
        double calculateTotalDistance(const std::vector<int>& tour);

    public:
        explicit MultiGoalPlanner(const Graph* graph);
        
        std::vector<int> planOptimalTour(int startId, const std::vector<int>& goals);
        std::vector<int> planOptimalTourWithReturn(int startId, const std::vector<int>& goals);
        std::vector<std::vector<int>> planIndividualPaths(int startId, const std::vector<int>& goals);

        void setOptimizationStrategy(MultiGoalStrategy strategy);
        void setPriorityGoals(const std::vector<std::pair<int, double>>& goalsWithPriority);

        double estimateTourCost(int startId, const std::vector<int>& goals);
        std::vector<int> getOptimalVisitationOrder(int startId,const std::vector<int>& goals);

        void setTimeLimit(double maxSeconds);
        void enableApproximation(bool enable);
};  
#pragma once
#include "../core/Graph.hpp"
#include <vector>

enum class LocalPlanningMethod
{
    DYNAMIC_WINDOW_APPROACH,
    POTENTIAL_FIELD,
    PURE_PURSUIT,
    MODEL_PREDICTIVE_CONTROL
};

class LocalPathPlanner
{
    private:
        const Graph* environment;
        LocalPlanningMethod planningMethod;
        double lookaheadDistance;
        double safetyMargin;

        std::vector<int> dynamicWindowApproach(const std::vector<int>& globalPath, int currentPosition, const std::vector<int>& obstacles);
        std::vector<int> potentialFieldMethod(const std::vector<int>& globalPath, int currentPosition, const std::vector<int>& obstacles);
        std::vector<int> purePursuitMethod(const std::vector<int>& globalPath, int currentPosition);

    public:
        explicit LocalPathPlanner(const Graph* graph);

        std::vector<int> generateLocalPath(const std::vector<int>& globalPath, int currentPosition, const std::vector<int>& dynamicObstacles);
        std::vector<int> avoidObstacles(const std::vector<int> plannedPath, const std::vector<int>& obstacles);
        std::vector<int> smoothPath(const std::vector<int>& roughPath);

        void setLookaheadDistance(double distance);
        void setSafetyMargin(double margin);
        void setPlanningMethod(LocalPlanningMethod method);

        bool isPathSafe(const std::vector<int>& path, const std::vector<int>& obstacles) const;
        double calculateClearance(int nodeId, const std::vector<int>& obstacles) const;
};
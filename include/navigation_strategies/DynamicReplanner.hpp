#pragma once
#include "core/Graph.hpp"
#include "GlobalPathPlanner.hpp"
#include <vector>
#include <chrono>

class DynamicReplanner
{
    private:
        const Graph* environment;
        std::unique_ptr<GlobalPathPlanner> globalPathPlanner;
        std::vector<int> currentPath;
        std::vector<int> knownObstacles;
        std::chrono::steady_clock::time_point lastReplanTime;
        double replanningInterval;

        bool requiresReplanning(const std::vector<int>& newObstacles) const;
        void updateObstacleMap(const std::vector<int>& newObstacles);

    public:
        explicit DynamicReplanner(const Graph* graph);

        std::vector<int> updateAndReplan(int currentPosition, int goalId, const std::vector<int>& detectedObstacles);
        void setInitialPath(const std::vector<int>& initialPath);
        bool shouldReplan(const std::vector<int>& sensorData) const;

        void setReplanningInterval(double intervalSeconds);
        void forceReplan();
        std::vector<int> getCurrentPath() const;

        void addPersistentObstacle(int nodeId);
        void removePersistentObstacle(int nodeId);
        void clearObstacles();

        double getReplanningFrequency() const;
        size_t getReplanningCount() const;
};
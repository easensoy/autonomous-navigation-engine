#pragma once
#include "../core/Graph.hpp"
#include <vector>
#include <functional>
#include <chrono>

enum class ExecutionState {
    IDLE,
    EXECUTING,
    PAUSED,
    COMPLETED,
    ERROR,
    EMERGENCY_STOP
};

class PathExecutor {
private:
    const Graph* environment;
    std::vector<int> executionPath;
    size_t currentWaypointIndex;
    ExecutionState currentState;
    
    std::function<void(int)> waypointReachedCallback;
    std::function<void(const std::string&)> errorCallback;
    std::function<bool(int)> safetyCheckCallback;
    
    std::chrono::steady_clock::time_point executionStartTime;
    double executionSpeed;

public:
    explicit PathExecutor(const Graph* graph);
    
    bool loadPath(const std::vector<int>& path);
    bool startExecution();
    void pauseExecution();
    void resumeExecution();
    void stopExecution();
    void emergencyStop();
    
    ExecutionState getExecutionState() const;
    int getCurrentWaypoint() const;
    size_t getRemainingWaypoints() const;
    double getExecutionProgress() const;
    
    void setWaypointReachedCallback(std::function<void(int)> callback);
    void setErrorCallback(std::function<void(const std::string&)> callback);
    void setSafetyCheckCallback(std::function<bool(int)> callback);
    
    bool executeNextStep();
    void setExecutionSpeed(double speed);
    std::chrono::duration<double> getExecutionTime() const;
    
    void skipToWaypoint(size_t waypointIndex);
    bool insertWaypoint(int nodeId, size_t position);
    void validatePathExecution() const;
};
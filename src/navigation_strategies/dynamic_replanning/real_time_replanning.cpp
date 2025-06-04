#include "navigation_strategies/DynamicReplanner.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/BFS.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

class RealTimeReplanner {
private:
    struct TimedObstacle {
        int nodeId;
        std::chrono::steady_clock::time_point detectionTime;
        double severity;
        bool isPersistent;
        
        TimedObstacle(int id, double sev = 1.0, bool persistent = false) 
            : nodeId(id), detectionTime(std::chrono::steady_clock::now()), 
              severity(sev), isPersistent(persistent) {}
    };
    
    struct ReplanningTask {
        int currentPosition;
        int goalPosition;
        std::vector<int> detectedObstacles;
        std::chrono::steady_clock::time_point requestTime;
        int priority;
        
        ReplanningTask(int curr, int goal, const std::vector<int>& obstacles, int prio = 0)
            : currentPosition(curr), goalPosition(goal), detectedObstacles(obstacles),
              requestTime(std::chrono::steady_clock::now()), priority(prio) {}
        
        bool operator<(const ReplanningTask& other) const {
            return priority < other.priority; // Higher priority = higher number
        }
    };
    
    const Graph* graph;
    std::unique_ptr<AStar> aStar;
    std::unique_ptr<BFS> bfs;
    
    // Threading and synchronization
    std::atomic<bool> isRunning;
    std::atomic<bool> replanningInProgress;
    std::mutex pathMutex;
    std::mutex obstacleMutex;
    std::mutex taskMutex;
    
    // Current state
    std::vector<int> currentPath;
    std::vector<int> backupPath;
    int currentPosition;
    int targetGoal;
    
    // Obstacle management
    std::unordered_map<int, TimedObstacle> activeObstacles;
    std::vector<int> emergencyObstacles;
    
    // Replanning configuration
    double maxReplanningTime;
    double obstacleDecayTime;
    double emergencyReplanThreshold;
    bool useBackupPaths;
    
    // Performance tracking
    std::chrono::steady_clock::time_point lastReplanTime;
    size_t replanningCount;
    double averageReplanTime;
    std::vector<double> replanTimes;
    
    // Task queue for prioritized replanning
    std::priority_queue<ReplanningTask> replanningTasks;
    
    std::vector<int> executeEmergencyReplan(int currentPos, int goal, const std::vector<int>& obstacles) {
        std::cout << "[REALTIME] EMERGENCY REPLAN from " << currentPos << " to " << goal << std::endl;
        
        auto startTime = std::chrono::steady_clock::now();
        
        // Use BFS for emergency replanning (faster but suboptimal)
        std::vector<int> emergencyPath = bfs->findPath(currentPos, goal);
        
        if (emergencyPath.empty()) {
            // If BFS fails, try A* with reduced search space
            std::cout << "[REALTIME] BFS failed, attempting limited A* search" << std::endl;
            emergencyPath = performLimitedAStar(currentPos, goal, obstacles);
        }
        
        auto endTime = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        
        std::cout << "[REALTIME] Emergency replan completed in " << duration.count() << "ms" << std::endl;
        
        return emergencyPath;
    }
    
    std::vector<int> performLimitedAStar(int startPos, int goal, const std::vector<int>& obstacles) {
        // Implement a time-limited A* search for emergency situations
        std::cout << "[REALTIME] Performing limited A* search" << std::endl;
        
        auto searchStart = std::chrono::steady_clock::now();
        const auto maxSearchTime = std::chrono::milliseconds(static_cast<long>(emergencyReplanThreshold * 1000));
        
        std::vector<int> path;
        
        try {
            // This is a simplified implementation - in practice, you'd modify A* to be interruptible
            path = aStar->findPath(startPos, goal);
            
            // Validate path doesn't go through new obstacles
            if (!path.empty() && pathContainsObstacles(path, obstacles)) {
                std::cout << "[REALTIME] A* path contains obstacles, attempting alternative" << std::endl;
                path = findAlternativeRoute(startPos, goal, obstacles);
            }
        } catch (const std::exception& e) {
            std::cout << "[REALTIME] Limited A* search failed: " << e.what() << std::endl;
        }
        
        auto searchEnd = std::chrono::steady_clock::now();
        auto searchDuration = std::chrono::duration_cast<std::chrono::milliseconds>(searchEnd - searchStart);
        
        if (searchDuration > maxSearchTime) {
            std::cout << "[REALTIME] Search exceeded time limit (" << searchDuration.count() << "ms)" << std::endl;
        }
        
        return path;
    }
    
    std::vector<int> findAlternativeRoute(int startPos, int goal, const std::vector<int>& obstacles) {
        std::cout << "[REALTIME] Finding alternative route avoiding obstacles" << std::endl;
        
        // Create obstacle set for quick lookup
        std::unordered_set<int> obstacleSet(obstacles.begin(), obstacles.end());
        
        // Try to find a path that goes around obstacles
        std::vector<int> neighbors = graph->getNeighbors(startPos);
        
        for (int neighbor : neighbors) {
            if (obstacleSet.find(neighbor) == obstacleSet.end()) {
                std::vector<int> alternatePath = bfs->findPath(neighbor, goal);
                if (!alternatePath.empty() && !pathContainsObstacles(alternatePath, obstacles)) {
                    // Prepend start position
                    alternatePath.insert(alternatePath.begin(), startPos);
                    std::cout << "[REALTIME] Found alternative route with " << alternatePath.size() << " nodes" << std::endl;
                    return alternatePath;
                }
            }
        }
        
        std::cout << "[REALTIME] No alternative route found" << std::endl;
        return {};
    }
    
    bool pathContainsObstacles(const std::vector<int>& path, const std::vector<int>& obstacles) const {
        std::unordered_set<int> obstacleSet(obstacles.begin(), obstacles.end());
        
        for (int nodeId : path) {
            if (obstacleSet.find(nodeId) != obstacleSet.end()) {
                return true;
            }
        }
        
        return false;
    }
    
    void updateObstacleState() {
        std::lock_guard<std::mutex> lock(obstacleMutex);
        
        auto currentTime = std::chrono::steady_clock::now();
        auto it = activeObstacles.begin();
        
        while (it != activeObstacles.end()) {
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                currentTime - it->second.detectionTime);
            
            if (!it->second.isPersistent && duration.count() > obstacleDecayTime) {
                std::cout << "[REALTIME] Obstacle " << it->first << " has expired" << std::endl;
                it = activeObstacles.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    std::vector<int> getActiveObstacleList() const {
        std::lock_guard<std::mutex> lock(obstacleMutex);
        
        std::vector<int> obstacles;
        for (const auto& pair : activeObstacles) {
            obstacles.push_back(pair.first);
        }
        
        return obstacles;
    }
    
    void updatePerformanceMetrics(double replanTime) {
        replanTimes.push_back(replanTime);
        
        if (replanTimes.size() > 100) {
            replanTimes.erase(replanTimes.begin());
        }
        
        double sum = 0.0;
        for (double time : replanTimes) {
            sum += time;
        }
        averageReplanTime = sum / replanTimes.size();
        
        replanningCount++;
    }
    
    bool isReplanningRequired(const std::vector<int>& newObstacles) const {
        std::lock_guard<std::mutex> lock(pathMutex);
        
        if (currentPath.empty()) {
            return true;
        }
        
        // Check if any new obstacles are on current path
        std::unordered_set<int> pathNodes(currentPath.begin(), currentPath.end());
        
        for (int obstacle : newObstacles) {
            if (pathNodes.find(obstacle) != pathNodes.end()) {
                return true;
            }
        }
        
        return false;
    }
    
    double calculateReplanUrgency(const std::vector<int>& obstacles) const {
        if (currentPath.empty()) {
            return 1.0; // Maximum urgency if no path
        }
        
        // Calculate how close obstacles are to current position
        double minDistance = std::numeric_limits<double>::infinity();
        
        for (int obstacle : obstacles) {
            // Find distance to current position
            auto it = std::find(currentPath.begin(), currentPath.end(), obstacle);
            if (it != currentPath.end()) {
                size_t distance = std::distance(currentPath.begin(), it);
                minDistance = std::min(minDistance, static_cast<double>(distance));
            }
        }
        
        if (minDistance == std::numeric_limits<double>::infinity()) {
            return 0.0; // No urgency if obstacles not on path
        }
        
        // Urgency decreases with distance
        return std::max(0.0, 1.0 - (minDistance / currentPath.size()));
    }
    
public:
    RealTimeReplanner(const Graph* environment) 
        : graph(environment), isRunning(false), replanningInProgress(false),
          currentPosition(-1), targetGoal(-1), maxReplanningTime(0.1),
          obstacleDecayTime(30.0), emergencyReplanThreshold(0.05),
          useBackupPaths(true), replanningCount(0), averageReplanTime(0.0) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        aStar = std::make_unique<AStar>(graph);
        bfs = std::make_unique<BFS>(graph);
        
        std::cout << "[REALTIME] Real-time replanner initialized" << std::endl;
        std::cout << "[REALTIME] Max replanning time: " << maxReplanningTime << "s" << std::endl;
        std::cout << "[REALTIME] Emergency threshold: " << emergencyReplanThreshold << "s" << std::endl;
    }
    
    std::vector<int> performRealTimeReplan(int currentPos, int goal, const std::vector<int>& obstacles) {
        auto startTime = std::chrono::steady_clock::now();
        
        std::cout << "[REALTIME] Real-time replanning from " << currentPos 
                  << " to " << goal << " with " << obstacles.size() << " obstacles" << std::endl;
        
        // Update current state
        {
            std::lock_guard<std::mutex> lock(pathMutex);
            currentPosition = currentPos;
            targetGoal = goal;
        }
        
        // Add new obstacles to active set
        {
            std::lock_guard<std::mutex> lock(obstacleMutex);
            for (int obstacle : obstacles) {
                activeObstacles[obstacle] = TimedObstacle(obstacle, 1.0, false);
            }
        }
        
        // Update obstacle state (remove expired obstacles)
        updateObstacleState();
        
        // Get current obstacle list
        std::vector<int> allObstacles = getActiveObstacleList();
        
        // Determine urgency
        double urgency = calculateReplanUrgency(allObstacles);
        bool isEmergency = urgency > 0.8; // High urgency threshold
        
        std::vector<int> newPath;
        
        if (isEmergency) {
            newPath = executeEmergencyReplan(currentPos, goal, allObstacles);
        } else {
            // Normal replanning with A*
            replanningInProgress.store(true);
            
            try {
                newPath = aStar->findPath(currentPos, goal);
                
                // Validate path doesn't contain obstacles
                if (pathContainsObstacles(newPath, allObstacles)) {
                    std::cout << "[REALTIME] Path contains obstacles, finding alternative" << std::endl;
                    newPath = findAlternativeRoute(currentPos, goal, allObstacles);
                }
            } catch (const std::exception& e) {
                std::cout << "[REALTIME] Normal replan failed: " << e.what() << std::endl;
                newPath = executeEmergencyReplan(currentPos, goal, allObstacles);
            }
            
            replanningInProgress.store(false);
        }
        
        // Update current path
        {
            std::lock_guard<std::mutex> lock(pathMutex);
            if (!currentPath.empty() && useBackupPaths) {
                backupPath = currentPath; // Keep backup
            }
            currentPath = newPath;
        }
        
        auto endTime = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
        double replanTime = duration.count() / 1000.0; // Convert to milliseconds
        
        updatePerformanceMetrics(replanTime);
        lastReplanTime = endTime;
        
        std::cout << "[REALTIME] Replanning completed in " << replanTime << "ms" 
                  << " (urgency: " << urgency << ")" << std::endl;
        
        return newPath;
    }
    
    void addPersistentObstacle(int nodeId) {
        std::lock_guard<std::mutex> lock(obstacleMutex);
        activeObstacles[nodeId] = TimedObstacle(nodeId, 1.0, true);
        std::cout << "[REALTIME] Added persistent obstacle at node " << nodeId << std::endl;
    }
    
    void removePersistentObstacle(int nodeId) {
        std::lock_guard<std::mutex> lock(obstacleMutex);
        auto it = activeObstacles.find(nodeId);
        if (it != activeObstacles.end()) {
            activeObstacles.erase(it);
            std::cout << "[REALTIME] Removed persistent obstacle at node " << nodeId << std::endl;
        }
    }
    
    void clearAllObstacles() {
        std::lock_guard<std::mutex> lock(obstacleMutex);
        activeObstacles.clear();
        emergencyObstacles.clear();
        std::cout << "[REALTIME] Cleared all obstacles" << std::endl;
    }
    
    std::vector<int> getCurrentPath() const {
        std::lock_guard<std::mutex> lock(pathMutex);
        return currentPath;
    }
    
    std::vector<int> getBackupPath() const {
        std::lock_guard<std::mutex> lock(pathMutex);
        return backupPath;
    }
    
    void setMaxReplanningTime(double seconds) {
        maxReplanningTime = seconds;
        std::cout << "[REALTIME] Max replanning time set to " << seconds << "s" << std::endl;
    }
    
    void setEmergencyThreshold(double seconds) {
        emergencyReplanThreshold = seconds;
        std::cout << "[REALTIME] Emergency threshold set to " << seconds << "s" << std::endl;
    }
    
    void setObstacleDecayTime(double seconds) {
        obstacleDecayTime = seconds;
        std::cout << "[REALTIME] Obstacle decay time set to " << seconds << "s" << std::endl;
    }
    
    bool isCurrentlyReplanning() const {
        return replanningInProgress.load();
    }
    
    double getAverageReplanTime() const {
        return averageReplanTime;
    }
    
    size_t getReplanningCount() const {
        return replanningCount;
    }
    
    void printPerformanceStats() const {
        std::cout << "[REALTIME] Performance Statistics:" << std::endl;
        std::cout << "[REALTIME]   Total replannings: " << replanningCount << std::endl;
        std::cout << "[REALTIME]   Average replan time: " << averageReplanTime << "ms" << std::endl;
        std::cout << "[REALTIME]   Active obstacles: " << activeObstacles.size() << std::endl;
        std::cout << "[REALTIME]   Currently replanning: " << (replanningInProgress.load() ? "Yes" : "No") << std::endl;
        
        if (!replanTimes.empty()) {
            double minTime = *std::min_element(replanTimes.begin(), replanTimes.end());
            double maxTime = *std::max_element(replanTimes.begin(), replanTimes.end());
            std::cout << "[REALTIME]   Min replan time: " << minTime << "ms" << std::endl;
            std::cout << "[REALTIME]   Max replan time: " << maxTime << "ms" << std::endl;
        }
        
        auto currentTime = std::chrono::steady_clock::now();
        if (lastReplanTime.time_since_epoch().count() > 0) {
            auto timeSince = std::chrono::duration_cast<std::chrono::seconds>(
                currentTime - lastReplanTime);
            std::cout << "[REALTIME]   Time since last replan: " << timeSince.count() << "s" << std::endl;
        }
    }
    
    void enableBackupPaths(bool enable) {
        useBackupPaths = enable;
        std::cout << "[REALTIME] Backup paths " << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    std::vector<int> getActiveObstacles() const {
        return getActiveObstacleList();
    }
};
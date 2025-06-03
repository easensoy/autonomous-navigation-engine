#include "autonomous_navigator/NavigationCore.hpp"
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <queue>
#include <mutex>
#include <condition_variable>

class RealTimeNavigationManager {
private:
    NavigationCore* navigationCore;
    std::atomic<bool> realTimeActive;
    std::atomic<bool> shutdownRequested;
    mutable std::mutex updateQueueMutex;
    std::thread navigationThread;
    std::mutex navigationMutex;
    std::condition_variable navigationCondition;
    
    struct NavigationUpdate {
        std::vector<int> obstacles;
        std::chrono::steady_clock::time_point timestamp;
        int priority;
        bool requiresReplanning;
    };
    
    std::queue<NavigationUpdate> updateQueue;
    std::mutex updateQueueMutex;
    double updateFrequency;
    double replanningThreshold;
    std::chrono::steady_clock::time_point lastUpdateTime;
    std::chrono::steady_clock::time_point lastReplanTime;
    
public:
    explicit RealTimeNavigationManager(NavigationCore* navCore) 
        : navigationCore(navCore), realTimeActive(false), shutdownRequested(false),
          updateFrequency(10.0), replanningThreshold(0.5) {}
    
    ~RealTimeNavigationManager() {
        stopRealTimeNavigation();
    }
    
    bool startRealTimeNavigation() {
        std::lock_guard<std::mutex> lock(navigationMutex);
        
        if (realTimeActive.load()) {
            std::cout << "[REAL_TIME_NAV] Real-time navigation already active" << std::endl;
            return true;
        }
        
        std::cout << "[REAL_TIME_NAV] Starting real-time navigation system" << std::endl;
        
        shutdownRequested.store(false);
        realTimeActive.store(true);
        lastUpdateTime = std::chrono::steady_clock::now();
        lastReplanTime = lastUpdateTime;
        
        // Start navigation monitoring thread
        navigationThread = std::thread(&RealTimeNavigationManager::navigationLoop, this);
        
        std::cout << "[REAL_TIME_NAV] Real-time navigation system started successfully" << std::endl;
        return true;
    }
    
    void stopRealTimeNavigation() {
        std::lock_guard<std::mutex> lock(navigationMutex);
        
        if (!realTimeActive.load()) {
            return;
        }
        
        std::cout << "[REAL_TIME_NAV] Stopping real-time navigation system" << std::endl;
        
        shutdownRequested.store(true);
        realTimeActive.store(false);
        navigationCondition.notify_all();
        
        if (navigationThread.joinable()) {
            navigationThread.join();
        }
        
        // Clear pending updates
        std::lock_guard<std::mutex> queueLock(updateQueueMutex);
        while (!updateQueue.empty()) {
            updateQueue.pop();
        }
        
        std::cout << "[REAL_TIME_NAV] Real-time navigation system stopped" << std::endl;
    }
    
    void processObstacleUpdate(const std::vector<int>& newObstacles, int priority = 1) {
        if (!realTimeActive.load()) {
            std::cout << "[REAL_TIME_NAV] Cannot process update - real-time navigation not active" << std::endl;
            return;
        }
        
        std::cout << "[REAL_TIME_NAV] Processing obstacle update with " << newObstacles.size() 
                  << " obstacles (priority " << priority << ")" << std::endl;
        
        NavigationUpdate update;
        update.obstacles = newObstacles;
        update.timestamp = std::chrono::steady_clock::now();
        update.priority = priority;
        update.requiresReplanning = analyzeUpdateImpact(newObstacles);
        
        {
            std::lock_guard<std::mutex> lock(updateQueueMutex);
            updateQueue.push(update);
        }
        
        // Notify navigation thread of new update
        navigationCondition.notify_one();
        
        // Handle high-priority updates immediately
        if (priority >= 5 && update.requiresReplanning) {
            std::cout << "[REAL_TIME_NAV] High-priority update requires immediate replanning" << std::endl;
            triggerImmediateReplanning(newObstacles);
        }
    }
    
    void setUpdateFrequency(double frequency) {
        updateFrequency = frequency;
        std::cout << "[REAL_TIME_NAV] Update frequency set to " << frequency << " Hz" << std::endl;
    }
    
    void setReplanningThreshold(double threshold) {
        replanningThreshold = threshold;
        std::cout << "[REAL_TIME_NAV] Replanning threshold set to " << threshold << std::endl;
    }
    
    bool isRealTimeActive() const {
        return realTimeActive.load();
    }
    
    void executeRealTimeStep() {
        if (!realTimeActive.load()) {
            return;
        }
        
        auto currentTime = std::chrono::steady_clock::now();
        auto timeSinceLastUpdate = std::chrono::duration<double>(currentTime - lastUpdateTime);
        
        // Check if it's time for a navigation update
        double updateInterval = 1.0 / updateFrequency;
        if (timeSinceLastUpdate.count() >= updateInterval) {
            performNavigationUpdate();
            lastUpdateTime = currentTime;
        }
        
        // Execute navigation step if active
        if (navigationCore->isNavigationComplete()) {
            std::cout << "[REAL_TIME_NAV] Navigation completed, stopping real-time monitoring" << std::endl;
            stopRealTimeNavigation();
        } else {
            navigationCore->executeNavigationStep();
        }
    }
    
    void handleEmergencyObstacle(const std::vector<int>& criticalObstacles) {
        std::cout << "[REAL_TIME_NAV] EMERGENCY OBSTACLE DETECTED!" << std::endl;
        std::cout << "[REAL_TIME_NAV] Critical obstacles: " << criticalObstacles.size() << std::endl;
        
        // Immediately handle critical obstacles
        navigationCore->handleObstacles(criticalObstacles);
        
        // Trigger emergency replanning
        triggerImmediateReplanning(criticalObstacles);
        
        // Process as high-priority update
        processObstacleUpdate(criticalObstacles, 10); // Maximum priority
    }
    
    void updateNavigationParameters(NavigationMode mode, double maxExecutionTime = 0.1) {
        std::cout << "[REAL_TIME_NAV] Updating navigation parameters for real-time operation" << std::endl;
        
        navigationCore->setNavigationMode(mode);
        
        // Adjust parameters for real-time constraints
        if (maxExecutionTime > 0) {
            std::cout << "[REAL_TIME_NAV] Setting execution time constraint: " 
                      << maxExecutionTime << " seconds" << std::endl;
        }
        
        std::cout << "[REAL_TIME_NAV] Navigation parameters updated" << std::endl;
    }
    
    double getNavigationEfficiency() const {
        if (!realTimeActive.load()) {
            return 0.0;
        }
        
        // Calculate efficiency based on update processing rate
        auto currentTime = std::chrono::steady_clock::now();
        auto totalRunTime = std::chrono::duration<double>(currentTime - lastReplanTime);
        
        // Simulate efficiency calculation
        double targetUpdateRate = updateFrequency;
        double actualUpdateRate = 1.0 / std::chrono::duration<double>(currentTime - lastUpdateTime).count();
        
        return std::min(1.0, actualUpdateRate / targetUpdateRate);
    }
    
    void generateRealTimeReport() const {
        std::cout << "\n[REAL_TIME_NAV] === REAL-TIME NAVIGATION REPORT ===" << std::endl;
        
        std::cout << "[REAL_TIME_NAV] System status: " 
                  << (realTimeActive.load() ? "ACTIVE" : "INACTIVE") << std::endl;
        std::cout << "[REAL_TIME_NAV] Update frequency: " << updateFrequency << " Hz" << std::endl;
        std::cout << "[REAL_TIME_NAV] Replanning threshold: " << replanningThreshold << std::endl;
        std::cout << "[REAL_TIME_NAV] Navigation efficiency: " 
                  << (getNavigationEfficiency() * 100.0) << "%" << std::endl;
        
        {
            std::lock_guard<std::mutex> lock(updateQueueMutex);
            std::cout << "[REAL_TIME_NAV] Pending updates: " << updateQueue.size() << std::endl;
        }
        
        std::cout << "[REAL_TIME_NAV] Navigation status: " 
                  << navigationCore->getNavigationStatus() << std::endl;
        
        if (navigationCore->isNavigationComplete()) {
            std::cout << "[REAL_TIME_NAV] Navigation: COMPLETED" << std::endl;
        } else {
            std::cout << "[REAL_TIME_NAV] Remaining distance: " 
                      << navigationCore->getRemainingDistance() << std::endl;
        }
        
        std::cout << "[REAL_TIME_NAV] === END REPORT ===" << std::endl;
    }
    
private:
    void navigationLoop() {
        std::cout << "[REAL_TIME_NAV] Navigation monitoring loop started" << std::endl;
        
        while (!shutdownRequested.load() && realTimeActive.load()) {
            try {
                // Process pending updates
                processUpdateQueue();
                
                // Execute navigation step
                executeRealTimeStep();
                
                // Monitor navigation status
                monitorNavigationHealth();
                
                // Sleep based on update frequency
                auto sleepDuration = std::chrono::milliseconds(
                    static_cast<int>(1000.0 / updateFrequency));
                std::this_thread::sleep_for(sleepDuration);
                
            } catch (const std::exception& e) {
                std::cout << "[REAL_TIME_NAV] Navigation loop error: " << e.what() << std::endl;
                
                // Attempt to recover from error
                if (!recoverFromNavigationError(e.what())) {
                    std::cout << "[REAL_TIME_NAV] Fatal error, stopping real-time navigation" << std::endl;
                    break;
                }
            }
        }
        
        std::cout << "[REAL_TIME_NAV] Navigation monitoring loop stopped" << std::endl;
    }
    
    void processUpdateQueue() {
        std::lock_guard<std::mutex> lock(updateQueueMutex);
        
        while (!updateQueue.empty()) {
            NavigationUpdate update = updateQueue.front();
            updateQueue.pop();
            
            std::cout << "[REAL_TIME_NAV] Processing queued update with " 
                      << update.obstacles.size() << " obstacles" << std::endl;
            
            // Apply obstacle update to navigation core
            navigationCore->handleObstacles(update.obstacles);
            
            // Check if replanning is required
            if (update.requiresReplanning) {
                auto timeSinceLastReplan = std::chrono::steady_clock::now() - lastReplanTime;
                double replanInterval = std::chrono::duration<double>(timeSinceLastReplan).count();
                
                if (replanInterval >= replanningThreshold) {
                    std::cout << "[REAL_TIME_NAV] Triggering replanning due to significant obstacles" << std::endl;
                    performReplanning(update.obstacles);
                    lastReplanTime = std::chrono::steady_clock::now();
                }
            }
        }
    }
    
    bool analyzeUpdateImpact(const std::vector<int>& obstacles) {
        if (obstacles.empty()) {
            return false;
        }
        
        // Analyze if obstacles affect current navigation path
        std::vector<int> currentPath = navigationCore->getCurrentPath();
        
        if (currentPath.empty()) {
            return false; // No active path to affect
        }
        
        // Check if any obstacles intersect with current path
        for (int obstacle : obstacles) {
            for (int pathNode : currentPath) {
                if (obstacle == pathNode) {
                    std::cout << "[REAL_TIME_NAV] Obstacle " << obstacle 
                              << " blocks current path at node " << pathNode << std::endl;
                    return true; // Replanning required
                }
            }
        }
        
        // Check proximity to current position
        int currentPosition = navigationCore->getCurrentPosition();
        for (int obstacle : obstacles) {
            if (std::abs(obstacle - currentPosition) <= 2) { // Within 2 nodes
                std::cout << "[REAL_TIME_NAV] Obstacle " << obstacle 
                          << " detected near current position " << currentPosition << std::endl;
                return true; // Replanning recommended for safety
            }
        }
        
        return false; // No immediate impact
    }
    
    void performNavigationUpdate() {
        // Update navigation state based on current conditions
        std::string status = navigationCore->getNavigationStatus();
        
        if (status.find("ERROR") != std::string::npos) {
            std::cout << "[REAL_TIME_NAV] Navigation error detected: " << status << std::endl;
            handleNavigationError(status);
        }
        
        // Check navigation progress
        double remainingDistance = navigationCore->getRemainingDistance();
        if (remainingDistance < 0.1 && !navigationCore->isNavigationComplete()) {
            std::cout << "[REAL_TIME_NAV] Navigation nearing completion" << std::endl;
        }
        
        // Monitor execution performance
        monitorExecutionPerformance();
    }
    
    void triggerImmediateReplanning(const std::vector<int>& obstacles) {
        std::cout << "[REAL_TIME_NAV] Executing immediate replanning for " 
                  << obstacles.size() << " obstacles" << std::endl;
        
        // Pause current navigation
        navigationCore->pauseNavigation();
        
        // Apply obstacles
        navigationCore->handleObstacles(obstacles);
        
        // Resume navigation (this should trigger replanning internally)
        navigationCore->resumeNavigation();
        
        lastReplanTime = std::chrono::steady_clock::now();
        
        std::cout << "[REAL_TIME_NAV] Immediate replanning completed" << std::endl;
    }
    
    void performReplanning(const std::vector<int>& obstacles) {
        std::cout << "[REAL_TIME_NAV] Performing strategic replanning" << std::endl;
        
        // Store current navigation state
        std::vector<int> originalPath = navigationCore->getCurrentPath();
        int currentPosition = navigationCore->getCurrentPosition();
        
        // Apply obstacles and replan
        navigationCore->handleObstacles(obstacles);
        
        // Verify new path is valid
        std::vector<int> newPath = navigationCore->getCurrentPath();
        if (newPath.empty()) {
            std::cout << "[REAL_TIME_NAV] Replanning failed to find valid path" << std::endl;
            
            // Attempt emergency recovery
            navigationCore->emergencyStop();
        } else {
            std::cout << "[REAL_TIME_NAV] Replanning successful, new path length: " 
                      << newPath.size() << std::endl;
        }
    }
    
    void monitorNavigationHealth() {
        // Check system health indicators
        std::string navigationStatus = navigationCore->getNavigationStatus();
        
        if (navigationStatus.find("EMERGENCY") != std::string::npos) {
            std::cout << "[REAL_TIME_NAV] Emergency condition detected in navigation core" << std::endl;
            handleEmergencyCondition();
        }
        
        // Monitor resource usage and performance
        double efficiency = getNavigationEfficiency();
        if (efficiency < 0.5) {
            std::cout << "[REAL_TIME_NAV] Performance warning: Navigation efficiency low (" 
                      << (efficiency * 100.0) << "%)" << std::endl;
            
            // Adjust update frequency to improve performance
            if (updateFrequency > 5.0) {
                updateFrequency *= 0.9; // Reduce frequency by 10%
                std::cout << "[REAL_TIME_NAV] Reduced update frequency to " 
                          << updateFrequency << " Hz" << std::endl;
            }
        }
    }
    
    void handleNavigationError(const std::string& errorStatus) {
        std::cout << "[REAL_TIME_NAV] Handling navigation error: " << errorStatus << std::endl;
        
        // Attempt error recovery strategies
        if (errorStatus.find("PATH") != std::string::npos) {
            std::cout << "[REAL_TIME_NAV] Path-related error, attempting replanning" << std::endl;
            performReplanning({}); // Replan without additional obstacles
        } else if (errorStatus.find("OBSTACLE") != std::string::npos) {
            std::cout << "[REAL_TIME_NAV] Obstacle-related error, clearing obstacle data" << std::endl;
            navigationCore->handleObstacles({}); // Clear obstacles
        } else {
            std::cout << "[REAL_TIME_NAV] Unknown error type, initiating full restart" << std::endl;
            restartNavigation();
        }
    }
    
    void handleEmergencyCondition() {
        std::cout << "[REAL_TIME_NAV] EMERGENCY CONDITION - Implementing safety protocols" << std::endl;
        
        // Immediately stop real-time processing
        realTimeActive.store(false);
        
        // Ensure navigation core is in safe state
        navigationCore->emergencyStop();
        
        std::cout << "[REAL_TIME_NAV] Emergency safety protocols activated" << std::endl;
    }
    
    bool recoverFromNavigationError(const std::string& error) {
        std::cout << "[REAL_TIME_NAV] Attempting recovery from error: " << error << std::endl;
        
        try {
            // Pause navigation for error recovery
            navigationCore->pauseNavigation();
            
            // Wait brief period for system stabilization
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Resume navigation
            navigationCore->resumeNavigation();
            
            std::cout << "[REAL_TIME_NAV] Error recovery successful" << std::endl;
            return true;
            
        } catch (const std::exception& recoveryError) {
            std::cout << "[REAL_TIME_NAV] Error recovery failed: " << recoveryError.what() << std::endl;
            return false;
        }
    }
    
    void restartNavigation() {
        std::cout << "[REAL_TIME_NAV] Restarting navigation system" << std::endl;
        
        // Stop current navigation
        navigationCore->stopNavigation();
        
        // Clear any pending updates
        {
            std::lock_guard<std::mutex> lock(updateQueueMutex);
            while (!updateQueue.empty()) {
                updateQueue.pop();
            }
        }
        
        // Reset timing
        lastUpdateTime = std::chrono::steady_clock::now();
        lastReplanTime = lastUpdateTime;
        
        std::cout << "[REAL_TIME_NAV] Navigation system restart completed" << std::endl;
    }
    
    void monitorExecutionPerformance() {
        // Monitor execution timing and throughput
        auto currentTime = std::chrono::steady_clock::now();
        auto executionInterval = std::chrono::duration<double>(currentTime - lastUpdateTime);
        
        double expectedInterval = 1.0 / updateFrequency;
        double actualInterval = executionInterval.count();
        
        if (actualInterval > expectedInterval * 1.5) {
            std::cout << "[REAL_TIME_NAV] Performance warning: Execution interval " 
                      << actualInterval << "s exceeds target " << expectedInterval << "s" << std::endl;
        }
    }
};
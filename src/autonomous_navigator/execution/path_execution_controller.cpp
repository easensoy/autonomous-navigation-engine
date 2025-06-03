#include "autonomous_navigator/PathExecutor.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <queue>
#include <thread>

class PathExecutionController {
private:
    PathExecutor* executor;
    bool precisionModeEnabled;
    double waypointTolerance;
    double executionSpeedMultiplier;
    std::vector<std::chrono::steady_clock::time_point> waypointTimestamps;
    std::vector<double> segmentSpeeds;
    std::queue<int> waypointQueue;
    bool adaptiveSpeedEnabled;
    
public:
    explicit PathExecutionController(PathExecutor* pathExecutor) 
        : executor(pathExecutor), precisionModeEnabled(false), waypointTolerance(0.1),
          executionSpeedMultiplier(1.0), adaptiveSpeedEnabled(true) {}
    
    bool initializeExecution(const std::vector<int>& path) {
        std::cout << "[EXECUTION_CONTROLLER] Initializing path execution controller" << std::endl;
        
        if (path.empty()) {
            std::cout << "[EXECUTION_CONTROLLER] Cannot initialize with empty path" << std::endl;
            return false;
        }
        
        // Load path into executor
        if (!executor->loadPath(path)) {
            std::cout << "[EXECUTION_CONTROLLER] Failed to load path into executor" << std::endl;
            return false;
        }
        
        // Initialize execution tracking
        waypointTimestamps.clear();
        waypointTimestamps.resize(path.size());
        segmentSpeeds.clear();
        segmentSpeeds.resize(path.size(), 1.0);
        
        // Clear waypoint queue and populate with path
        while (!waypointQueue.empty()) {
            waypointQueue.pop();
        }
        
        for (int waypoint : path) {
            waypointQueue.push(waypoint);
        }
        
        // Calculate optimal segment speeds if adaptive speed is enabled
        if (adaptiveSpeedEnabled) {
            calculateAdaptiveSegmentSpeeds(path);
        }
        
        std::cout << "[EXECUTION_CONTROLLER] Path execution controller initialized with " 
                  << path.size() << " waypoints" << std::endl;
        
        return true;
    }
    
    bool executeControlledPath() {
        std::cout << "[EXECUTION_CONTROLLER] Starting controlled path execution" << std::endl;
        
        if (executor->getExecutionState() == ExecutionState::EMERGENCY_STOP) {
            std::cout << "[EXECUTION_CONTROLLER] Cannot execute - emergency stop active" << std::endl;
            return false;
        }
        
        // Set up execution callbacks
        setupExecutionCallbacks();
        
        // Start execution with controller oversight
        if (!executor->startExecution()) {
            std::cout << "[EXECUTION_CONTROLLER] Failed to start path execution" << std::endl;
            return false;
        }
        
        // Monitor and control execution progress
        bool executionSuccess = monitorExecutionProgress();
        
        if (executionSuccess) {
            std::cout << "[EXECUTION_CONTROLLER] Controlled path execution completed successfully" << std::endl;
        } else {
            std::cout << "[EXECUTION_CONTROLLER] Controlled path execution encountered issues" << std::endl;
        }
        
        return executionSuccess;
    }
    
    void pauseExecution() {
        std::cout << "[EXECUTION_CONTROLLER] Pausing controlled execution" << std::endl;
        executor->pauseExecution();
        
        // Record pause timestamp for current waypoint
        size_t currentIndex = executor->getCurrentWaypoint();
        if (currentIndex < waypointTimestamps.size()) {
            waypointTimestamps[currentIndex] = std::chrono::steady_clock::now();
        }
    }
    
    void resumeExecution() {
        std::cout << "[EXECUTION_CONTROLLER] Resuming controlled execution" << std::endl;
        
        // Adjust execution speed based on pause duration if needed
        if (adaptiveSpeedEnabled) {
            adjustSpeedAfterPause();
        }
        
        executor->resumeExecution();
    }
    
    void setExecutionPrecision(bool precisionMode, double tolerance = 0.1) {
        precisionModeEnabled = precisionMode;
        waypointTolerance = tolerance;
        
        std::cout << "[EXECUTION_CONTROLLER] Precision mode " 
                  << (precisionMode ? "enabled" : "disabled");
        if (precisionMode) {
            std::cout << " with tolerance " << tolerance;
        }
        std::cout << std::endl;
        
        // Adjust execution speed based on precision requirements
        if (precisionMode) {
            executionSpeedMultiplier = 0.5; // Slower for precision
        } else {
            executionSpeedMultiplier = 1.0; // Normal speed
        }
        
        updateExecutionSpeed();
    }
    
    void setAdaptiveSpeed(bool enabled) {
        adaptiveSpeedEnabled = enabled;
        std::cout << "[EXECUTION_CONTROLLER] Adaptive speed control " 
                  << (enabled ? "enabled" : "disabled") << std::endl;
    }
    
    bool insertWaypointDynamic(int nodeId, size_t position) {
        std::cout << "[EXECUTION_CONTROLLER] Inserting waypoint " << nodeId 
                  << " at position " << position << std::endl;
        
        if (!executor->insertWaypoint(nodeId, position)) {
            std::cout << "[EXECUTION_CONTROLLER] Failed to insert waypoint" << std::endl;
            return false;
        }
        
        // Update controller tracking structures
        if (position < waypointTimestamps.size()) {
            waypointTimestamps.insert(waypointTimestamps.begin() + position, 
                                    std::chrono::steady_clock::time_point{});
            segmentSpeeds.insert(segmentSpeeds.begin() + position, 1.0);
        }
        
        std::cout << "[EXECUTION_CONTROLLER] Waypoint insertion completed" << std::endl;
        return true;
    }
    
    void skipToWaypointControlled(size_t waypointIndex) {
        std::cout << "[EXECUTION_CONTROLLER] Skipping to waypoint index " << waypointIndex << std::endl;
        
        executor->skipToWaypoint(waypointIndex);
        
        // Update controller state
        for (size_t i = 0; i < waypointIndex && i < waypointTimestamps.size(); ++i) {
            waypointTimestamps[i] = std::chrono::steady_clock::now();
        }
        
        // Clear skipped waypoints from queue
        std::queue<int> newQueue;
        size_t skipCount = 0;
        while (!waypointQueue.empty() && skipCount < waypointIndex) {
            waypointQueue.pop();
            skipCount++;
        }
    }
    
    double calculateExecutionEfficiency() const {
        if (waypointTimestamps.empty()) {
            return 0.0;
        }
        
        size_t completedWaypoints = 0;
        auto currentTime = std::chrono::steady_clock::now();
        auto executionStartTime = executor->getExecutionTime();
        
        for (const auto& timestamp : waypointTimestamps) {
            if (timestamp != std::chrono::steady_clock::time_point{}) {
                completedWaypoints++;
            }
        }
        
        if (completedWaypoints == 0) {
            return 0.0;
        }
        
        double actualTime = std::chrono::duration<double>(currentTime - 
                           (currentTime - executionStartTime)).count();
        double expectedTime = completedWaypoints * (1.0 / executor->getExecutionSpeed());
        
        return expectedTime > 0 ? std::min(1.0, expectedTime / actualTime) : 0.0;
    }
    
    void generateExecutionReport() const {
        std::cout << "\n[EXECUTION_CONTROLLER] === EXECUTION REPORT ===" << std::endl;
        
        ExecutionState state = executor->getExecutionState();
        std::cout << "[EXECUTION_CONTROLLER] Final execution state: ";
        switch (state) {
            case ExecutionState::IDLE: std::cout << "IDLE"; break;
            case ExecutionState::EXECUTING: std::cout << "EXECUTING"; break;
            case ExecutionState::PAUSED: std::cout << "PAUSED"; break;
            case ExecutionState::COMPLETED: std::cout << "COMPLETED"; break;
            case ExecutionState::ERROR: std::cout << "ERROR"; break;
            case ExecutionState::EMERGENCY_STOP: std::cout << "EMERGENCY_STOP"; break;
        }
        std::cout << std::endl;
        
        std::cout << "[EXECUTION_CONTROLLER] Total waypoints: " << waypointTimestamps.size() << std::endl;
        std::cout << "[EXECUTION_CONTROLLER] Execution progress: " 
                  << (executor->getExecutionProgress() * 100.0) << "%" << std::endl;
        std::cout << "[EXECUTION_CONTROLLER] Total execution time: " 
                  << executor->getExecutionTime().count() << " seconds" << std::endl;
        std::cout << "[EXECUTION_CONTROLLER] Execution efficiency: " 
                  << (calculateExecutionEfficiency() * 100.0) << "%" << std::endl;
        std::cout << "[EXECUTION_CONTROLLER] Precision mode: " 
                  << (precisionModeEnabled ? "Enabled" : "Disabled") << std::endl;
        std::cout << "[EXECUTION_CONTROLLER] Adaptive speed: " 
                  << (adaptiveSpeedEnabled ? "Enabled" : "Disabled") << std::endl;
        
        analyzeWaypointPerformance();
        
        std::cout << "[EXECUTION_CONTROLLER] === END REPORT ===" << std::endl;
    }
    
    std::vector<double> getWaypointExecutionTimes() const {
        std::vector<double> executionTimes;
        
        for (size_t i = 1; i < waypointTimestamps.size(); ++i) {
            if (waypointTimestamps[i] != std::chrono::steady_clock::time_point{} &&
                waypointTimestamps[i-1] != std::chrono::steady_clock::time_point{}) {
                
                auto duration = waypointTimestamps[i] - waypointTimestamps[i-1];
                executionTimes.push_back(std::chrono::duration<double>(duration).count());
            } else {
                executionTimes.push_back(0.0);
            }
        }
        
        return executionTimes;
    }
    
private:
    void setupExecutionCallbacks() {
        // Set waypoint reached callback
        executor->setWaypointReachedCallback([this](int waypointId) {
            handleWaypointReached(waypointId);
        });
        
        // Set error callback
        executor->setErrorCallback([this](const std::string& error) {
            handleExecutionError(error);
        });
        
        // Set safety check callback
        executor->setSafetyCheckCallback([this](int nodeId) {
            return performSafetyCheck(nodeId);
        });
    }
    
    void handleWaypointReached(int waypointId) {
        auto currentTime = std::chrono::steady_clock::now();
        size_t currentIndex = executor->getCurrentWaypoint();
        
        std::cout << "[EXECUTION_CONTROLLER] Waypoint " << waypointId 
                  << " reached (index " << currentIndex << ")" << std::endl;
        
        // Record timestamp
        if (currentIndex < waypointTimestamps.size()) {
            waypointTimestamps[currentIndex] = currentTime;
        }
        
        // Remove from queue
        if (!waypointQueue.empty()) {
            waypointQueue.pop();
        }
        
        // Adjust speed for next segment if adaptive mode is enabled
        if (adaptiveSpeedEnabled && currentIndex + 1 < segmentSpeeds.size()) {
            double nextSegmentSpeed = segmentSpeeds[currentIndex + 1] * executionSpeedMultiplier;
            executor->setExecutionSpeed(nextSegmentSpeed);
        }
        
        // Perform precision check if enabled
        if (precisionModeEnabled) {
            performPrecisionValidation(waypointId);
        }
    }
    
    void handleExecutionError(const std::string& error) {
        std::cout << "[EXECUTION_CONTROLLER] Execution error: " << error << std::endl;
        
        // Attempt error recovery
        if (attemptErrorRecovery(error)) {
            std::cout << "[EXECUTION_CONTROLLER] Error recovery successful, resuming execution" << std::endl;
        } else {
            std::cout << "[EXECUTION_CONTROLLER] Error recovery failed, stopping execution" << std::endl;
            executor->stopExecution();
        }
    }
    
    bool performSafetyCheck(int nodeId) {
        // Implement comprehensive safety checks
        std::cout << "[EXECUTION_CONTROLLER] Performing safety check for node " << nodeId << std::endl;
        
        // Simulated safety checks - in production, these would be real safety validations
        // Check 1: Node accessibility
        if (nodeId < 0) {
            std::cout << "[EXECUTION_CONTROLLER] Safety check failed: Invalid node ID" << std::endl;
            return false;
        }
        
        // Check 2: Environmental conditions (simulated)
        static int safetyCheckCounter = 0;
        safetyCheckCounter++;
        
        // Simulate occasional safety concerns
        if (safetyCheckCounter % 100 == 0) {
            std::cout << "[EXECUTION_CONTROLLER] Safety check warning: Environmental conditions monitored" << std::endl;
        }
        
        // Check 3: System status
        if (executor->getExecutionState() == ExecutionState::ERROR) {
            std::cout << "[EXECUTION_CONTROLLER] Safety check failed: System in error state" << std::endl;
            return false;
        }
        
        return true; // All safety checks passed
    }
    
    bool monitorExecutionProgress() {
        std::cout << "[EXECUTION_CONTROLLER] Monitoring execution progress" << std::endl;
        
        while (executor->getExecutionState() == ExecutionState::EXECUTING) {
            if (!executor->executeNextStep()) {
                ExecutionState currentState = executor->getExecutionState();
                
                if (currentState == ExecutionState::COMPLETED) {
                    std::cout << "[EXECUTION_CONTROLLER] Path execution completed successfully" << std::endl;
                    return true;
                } else if (currentState == ExecutionState::EMERGENCY_STOP) {
                    std::cout << "[EXECUTION_CONTROLLER] Execution stopped due to emergency" << std::endl;
                    return false;
                } else if (currentState == ExecutionState::ERROR) {
                    std::cout << "[EXECUTION_CONTROLLER] Execution stopped due to error" << std::endl;
                    return false;
                }
            }
            
            // Monitor execution performance
            monitorPerformanceMetrics();
            
            // Small delay to prevent tight loop
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        return executor->getExecutionState() == ExecutionState::COMPLETED;
    }
    
    void calculateAdaptiveSegmentSpeeds(const std::vector<int>& path) {
        std::cout << "[EXECUTION_CONTROLLER] Calculating adaptive segment speeds" << std::endl;
        
        for (size_t i = 0; i < path.size(); ++i) {
            double segmentSpeed = 1.0; // Base speed
            
            // Adjust speed based on path characteristics
            if (i < path.size() - 1) {
                // Analyze upcoming segment complexity
                int currentNode = path[i];
                int nextNode = path[i + 1];
                
                // Simulated complexity analysis
                double complexity = std::abs(nextNode - currentNode) / 100.0;
                segmentSpeed = 1.0 - (complexity * 0.3); // Reduce speed for complex segments
                segmentSpeed = std::max(0.2, std::min(1.5, segmentSpeed)); // Clamp speed
            }
            
            segmentSpeeds[i] = segmentSpeed;
        }
        
        std::cout << "[EXECUTION_CONTROLLER] Adaptive segment speeds calculated" << std::endl;
    }
    
    void updateExecutionSpeed() {
        double baseSpeed = 1.0;
        if (adaptiveSpeedEnabled) {
            size_t currentIndex = executor->getCurrentWaypoint();
            if (currentIndex < segmentSpeeds.size()) {
                baseSpeed = segmentSpeeds[currentIndex];
            }
        }
        
        double finalSpeed = baseSpeed * executionSpeedMultiplier;
        executor->setExecutionSpeed(finalSpeed);
        
        std::cout << "[EXECUTION_CONTROLLER] Updated execution speed to " << finalSpeed << std::endl;
    }
    
    void adjustSpeedAfterPause() {
        std::cout << "[EXECUTION_CONTROLLER] Adjusting speed after pause" << std::endl;
        
        // Gradually ramp up speed after pause to ensure smooth continuation
        executionSpeedMultiplier = 0.5; // Start slower
        updateExecutionSpeed();
        
        // Schedule speed restoration (in a real implementation, this would be more sophisticated)
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            executionSpeedMultiplier = 1.0;
            updateExecutionSpeed();
        }).detach();
    }
    
    void performPrecisionValidation(int waypointId) {
        std::cout << "[EXECUTION_CONTROLLER] Performing precision validation for waypoint " << waypointId << std::endl;
        
        // Simulated precision validation
        double precisionError = (rand() % 100) / 1000.0; // Random error up to 0.1
        
        if (precisionError > waypointTolerance) {
            std::cout << "[EXECUTION_CONTROLLER] Precision validation failed: error " 
                      << precisionError << " exceeds tolerance " << waypointTolerance << std::endl;
            
            // Implement precision correction
            executor->pauseExecution();
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Correction time
            executor->resumeExecution();
        } else {
            std::cout << "[EXECUTION_CONTROLLER] Precision validation passed" << std::endl;
        }
    }
    
    bool attemptErrorRecovery(const std::string& error) {
        std::cout << "[EXECUTION_CONTROLLER] Attempting error recovery for: " << error << std::endl;
        
        // Simulated error recovery logic
        if (error.find("safety") != std::string::npos) {
            std::cout << "[EXECUTION_CONTROLLER] Safety error detected, performing safety recovery" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return true;
        }
        
        if (error.find("waypoint") != std::string::npos) {
            std::cout << "[EXECUTION_CONTROLLER] Waypoint error detected, attempting path correction" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return true;
        }
        
        std::cout << "[EXECUTION_CONTROLLER] No recovery strategy available for this error type" << std::endl;
        return false;
    }
    
    void monitorPerformanceMetrics() {
        // Monitor execution efficiency
        double efficiency = calculateExecutionEfficiency();
        
        if (efficiency < 0.5) { // Below 50% efficiency
            std::cout << "[EXECUTION_CONTROLLER] Performance warning: Execution efficiency low (" 
                      << (efficiency * 100.0) << "%)" << std::endl;
            
            // Adjust execution parameters to improve efficiency
            if (adaptiveSpeedEnabled) {
                executionSpeedMultiplier *= 1.1; // Increase speed slightly
                updateExecutionSpeed();
            }
        }
    }
    
    void analyzeWaypointPerformance() const {
        std::cout << "[EXECUTION_CONTROLLER] Waypoint performance analysis:" << std::endl;
        
        std::vector<double> executionTimes = getWaypointExecutionTimes();
        
        if (executionTimes.empty()) {
            std::cout << "[EXECUTION_CONTROLLER] No waypoint timing data available" << std::endl;
            return;
        }
        
        double totalTime = std::accumulate(executionTimes.begin(), executionTimes.end(), 0.0);
        double averageTime = totalTime / executionTimes.size();
        
        auto minMaxTimes = std::minmax_element(executionTimes.begin(), executionTimes.end());
        
        std::cout << "[EXECUTION_CONTROLLER] Average waypoint time: " << averageTime << "s" << std::endl;
        std::cout << "[EXECUTION_CONTROLLER] Fastest waypoint time: " << *minMaxTimes.first << "s" << std::endl;
        std::cout << "[EXECUTION_CONTROLLER] Slowest waypoint time: " << *minMaxTimes.second << "s" << std::endl;
        
        // Identify performance bottlenecks
        size_t slowWaypoints = 0;
        for (double time : executionTimes) {
            if (time > averageTime * 1.5) {
                slowWaypoints++;
            }
        }
        
        std::cout << "[EXECUTION_CONTROLLER] Performance bottlenecks: " << slowWaypoints 
                  << " waypoints (" << (slowWaypoints * 100.0 / executionTimes.size()) << "%)" << std::endl;
    }
};
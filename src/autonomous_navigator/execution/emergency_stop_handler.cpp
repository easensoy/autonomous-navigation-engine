#include "autonomous_navigator/PathExecutor.hpp"
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

class EmergencyStopManager {
private:
    std::atomic<bool> emergencyStopTriggered;
    std::atomic<bool> emergencyStopActive;
    mutable std::mutex emergencyMutex;
    std::mutex emergencyMutex;
    std::condition_variable emergencyCondition;
    std::chrono::steady_clock::time_point emergencyTriggerTime;
    std::string emergencyReason;
    std::function<void(const std::string&)> emergencyCallback;
    bool autoRecoveryEnabled;
    double autoRecoveryDelay;
    
public:
    EmergencyStopManager() : emergencyStopTriggered(false), emergencyStopActive(false),
                           autoRecoveryEnabled(false), autoRecoveryDelay(5.0) {}
    
    void initiateEmergencyStop(const std::string& reason) {
        std::lock_guard<std::mutex> lock(emergencyMutex);
        
        if (emergencyStopActive.load()) {
            std::cout << "[EMERGENCY_STOP] Emergency stop already active, reason: " << emergencyReason << std::endl;
            return;
        }
        
        emergencyStopTriggered.store(true);
        emergencyStopActive.store(true);
        emergencyTriggerTime = std::chrono::steady_clock::now();
        emergencyReason = reason;
        
        std::cout << "[EMERGENCY_STOP] EMERGENCY STOP INITIATED!" << std::endl;
        std::cout << "[EMERGENCY_STOP] Reason: " << reason << std::endl;
        std::cout << "[EMERGENCY_STOP] All navigation operations halted immediately" << std::endl;
        
        if (emergencyCallback) {
            emergencyCallback(reason);
        }
        
        performImmediateStop();
        
        if (autoRecoveryEnabled) {
            std::thread([this]() { performAutoRecovery(); }).detach();
        }
    }
    
    bool isEmergencyStopActive() const {
        return emergencyStopActive.load();
    }
    
    bool isEmergencyStopTriggered() const {
        return emergencyStopTriggered.load();
    }
    
    void clearEmergencyStop() {
        std::lock_guard<std::mutex> lock(emergencyMutex);
        
        if (!emergencyStopActive.load()) {
            std::cout << "[EMERGENCY_STOP] No active emergency stop to clear" << std::endl;
            return;
        }
        
        emergencyStopTriggered.store(false);
        emergencyStopActive.store(false);
        emergencyReason.clear();
        
        std::cout << "[EMERGENCY_STOP] Emergency stop cleared - normal operations may resume" << std::endl;
        emergencyCondition.notify_all();
    }
    
    void waitForEmergencyClear() {
        std::unique_lock<std::mutex> lock(emergencyMutex);
        emergencyCondition.wait(lock, [this] { return !emergencyStopActive.load(); });
    }
    
    bool waitForEmergencyClear(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(emergencyMutex);
        return emergencyCondition.wait_for(lock, timeout, [this] { return !emergencyStopActive.load(); });
    }
    
    void setEmergencyCallback(std::function<void(const std::string&)> callback) {
        emergencyCallback = callback;
    }
    
    void enableAutoRecovery(bool enable, double delaySeconds = 5.0) {
        autoRecoveryEnabled = enable;
        autoRecoveryDelay = delaySeconds;
        
        std::cout << "[EMERGENCY_STOP] Auto recovery " << (enable ? "enabled" : "disabled");
        if (enable) {
            std::cout << " with " << delaySeconds << "s delay";
        }
        std::cout << std::endl;
    }
    
    std::string getEmergencyReason() const {
        std::lock_guard<std::mutex> lock(emergencyMutex);
        return emergencyReason;
    }
    
    double getEmergencyDuration() const {
        if (!emergencyStopActive.load()) {
            return 0.0;
        }
        
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration<double>(now - emergencyTriggerTime);
        return duration.count();
    }
    
    void checkEmergencyConditions(int currentWaypoint, const std::vector<int>& path,
                                std::function<bool(int)> safetyCheck) {
        if (emergencyStopActive.load()) {
            return; // Already in emergency state
        }
        
        // Safety check for current waypoint
        if (safetyCheck && !safetyCheck(currentWaypoint)) {
            initiateEmergencyStop("Safety check failed for current waypoint");
            return;
        }
        
        // Check path validity
        if (path.empty()) {
            initiateEmergencyStop("Empty path detected during execution");
            return;
        }
        
        // Check for waypoint index bounds
        if (static_cast<size_t>(currentWaypoint) >= path.size()) {
            initiateEmergencyStop("Current waypoint index out of bounds");
            return;
        }
        
        // Additional automated safety checks could be added here
        performSystemHealthCheck();
    }
    
private:
    void performImmediateStop() {
        std::cout << "[EMERGENCY_STOP] Performing immediate system halt..." << std::endl;
        
        // Stop all motion commands
        std::cout << "[EMERGENCY_STOP] Motion commands halted" << std::endl;
        
        // Engage safety mechanisms
        std::cout << "[EMERGENCY_STOP] Safety mechanisms engaged" << std::endl;
        
        // Log emergency state
        std::cout << "[EMERGENCY_STOP] Emergency state logged at: " 
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now().time_since_epoch()).count() 
                  << "ms" << std::endl;
    }
    
    void performAutoRecovery() {
        std::cout << "[EMERGENCY_STOP] Auto recovery initiated, waiting " 
                  << autoRecoveryDelay << " seconds..." << std::endl;
        
        std::this_thread::sleep_for(std::chrono::duration<double>(autoRecoveryDelay));
        
        // Perform system checks before recovery
        if (performRecoveryChecks()) {
            std::cout << "[EMERGENCY_STOP] System checks passed, clearing emergency stop" << std::endl;
            clearEmergencyStop();
        } else {
            std::cout << "[EMERGENCY_STOP] System checks failed, emergency stop remains active" << std::endl;
        }
    }
    
    bool performRecoveryChecks() {
        std::cout << "[EMERGENCY_STOP] Performing recovery system checks..." << std::endl;
        
        // Simulate system health checks
        std::cout << "[EMERGENCY_STOP] Checking system integrity..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "[EMERGENCY_STOP] Checking sensor functionality..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "[EMERGENCY_STOP] Checking navigation systems..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // All checks passed for simulation
        std::cout << "[EMERGENCY_STOP] All recovery checks completed successfully" << std::endl;
        return true;
    }
    
    void performSystemHealthCheck() {
        // This would contain real-time system health monitoring
        // For now, we simulate basic checks
        
        static int healthCheckCounter = 0;
        healthCheckCounter++;
        
        // Simulate occasional health issues
        if (healthCheckCounter % 1000 == 0) {
            std::cout << "[EMERGENCY_STOP] Periodic system health check performed" << std::endl;
        }
        
        // Simulate critical failure detection (very rare)
        if (healthCheckCounter % 50000 == 0) {
            initiateEmergencyStop("Periodic system health check detected anomaly");
        }
    }
};

// Global emergency stop manager
static std::unique_ptr<EmergencyStopManager> g_emergencyManager;

void PathExecutor::emergencyStop() {
    std::cout << "[PATH_EXECUTOR] Emergency stop requested" << std::endl;
    
    if (!g_emergencyManager) {
        g_emergencyManager = std::make_unique<EmergencyStopManager>();
    }
    
    // Set current state to emergency stop
    currentState = ExecutionState::EMERGENCY_STOP;
    
    // Trigger emergency stop with context
    std::string reason = "Manual emergency stop requested during path execution";
    if (currentWaypointIndex < executionPath.size()) {
        reason += " at waypoint " + std::to_string(currentWaypointIndex) + 
                  " (node " + std::to_string(executionPath[currentWaypointIndex]) + ")";
    }
    
    g_emergencyManager->initiateEmergencyStop(reason);
    
    // Call error callback if set
    if (errorCallback) {
        errorCallback("Emergency stop activated: " + reason);
    }
    
    std::cout << "[PATH_EXECUTOR] Path execution halted due to emergency stop" << std::endl;
}

bool PathExecutor::executeNextStep() {
    // Check for emergency stop before any execution
    if (g_emergencyManager && g_emergencyManager->isEmergencyStopActive()) {
        std::cout << "[PATH_EXECUTOR] Execution blocked by active emergency stop" << std::endl;
        currentState = ExecutionState::EMERGENCY_STOP;
        return false;
    }
    
    // Continue with normal execution logic
    if (currentState != ExecutionState::EXECUTING) {
        std::cout << "[PATH_EXECUTOR] Cannot execute step - not in executing state" << std::endl;
        return false;
    }
    
    if (currentWaypointIndex >= executionPath.size()) {
        std::cout << "[PATH_EXECUTOR] Path execution completed" << std::endl;
        currentState = ExecutionState::COMPLETED;
        return false;
    }
    
    // Perform safety check if callback is set
    int currentWaypoint = executionPath[currentWaypointIndex];
    if (safetyCheckCallback && !safetyCheckCallback(currentWaypoint)) {
        std::cout << "[PATH_EXECUTOR] Safety check failed for waypoint " << currentWaypoint << std::endl;
        
        if (!g_emergencyManager) {
            g_emergencyManager = std::make_unique<EmergencyStopManager>();
        }
        
        g_emergencyManager->initiateEmergencyStop("Safety check failure at waypoint " + 
                                                std::to_string(currentWaypoint));
        currentState = ExecutionState::EMERGENCY_STOP;
        return false;
    }
    
    // Automated emergency condition checks
    if (g_emergencyManager) {
        g_emergencyManager->checkEmergencyConditions(currentWaypoint, executionPath, safetyCheckCallback);
        
        if (g_emergencyManager->isEmergencyStopActive()) {
            currentState = ExecutionState::EMERGENCY_STOP;
            return false;
        }
    }
    
    // Execute the step
    std::cout << "[PATH_EXECUTOR] Executing step to waypoint " << currentWaypoint 
              << " (" << (currentWaypointIndex + 1) << "/" << executionPath.size() << ")" << std::endl;
    
    // Simulate step execution
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 / executionSpeed)));
    
    // Call waypoint reached callback
    if (waypointReachedCallback) {
        waypointReachedCallback(currentWaypoint);
    }
    
    currentWaypointIndex++;
    
    // Check if path is completed
    if (currentWaypointIndex >= executionPath.size()) {
        currentState = ExecutionState::COMPLETED;
        std::cout << "[PATH_EXECUTOR] Path execution completed successfully" << std::endl;
    }
    
    return true;
}

bool PathExecutor::startExecution() {
    // Initialize emergency manager if not already done
    if (!g_emergencyManager) {
        g_emergencyManager = std::make_unique<EmergencyStopManager>();
        
        // Set up emergency callback
        g_emergencyManager->setEmergencyCallback([this](const std::string& reason) {
            if (errorCallback) {
                errorCallback("Emergency stop: " + reason);
            }
        });
    }
    
    // Check for active emergency stop
    if (g_emergencyManager->isEmergencyStopActive()) {
        std::cout << "[PATH_EXECUTOR] Cannot start execution - emergency stop is active" << std::endl;
        std::cout << "[PATH_EXECUTOR] Emergency reason: " << g_emergencyManager->getEmergencyReason() << std::endl;
        return false;
    }
    
    if (executionPath.empty()) {
        std::cout << "[PATH_EXECUTOR] Cannot start execution - no path loaded" << std::endl;
        return false;
    }
    
    if (currentState == ExecutionState::EXECUTING) {
        std::cout << "[PATH_EXECUTOR] Execution already in progress" << std::endl;
        return true;
    }
    
    currentState = ExecutionState::EXECUTING;
    executionStartTime = std::chrono::steady_clock::now();
    currentWaypointIndex = 0;
    
    std::cout << "[PATH_EXECUTOR] Starting path execution with " << executionPath.size() 
              << " waypoints" << std::endl;
    
    return true;
}

void PathExecutor::stopExecution() {
    if (currentState == ExecutionState::EXECUTING) {
        currentState = ExecutionState::IDLE;
        std::cout << "[PATH_EXECUTOR] Path execution stopped" << std::endl;
    }
}

void PathExecutor::validatePathExecution() const {
    std::cout << "[PATH_EXECUTOR] Validating path execution state..." << std::endl;
    
    if (g_emergencyManager && g_emergencyManager->isEmergencyStopActive()) {
        std::cout << "[PATH_EXECUTOR] VALIDATION WARNING: Emergency stop is active" << std::endl;
        std::cout << "[PATH_EXECUTOR] Emergency reason: " << g_emergencyManager->getEmergencyReason() << std::endl;
        std::cout << "[PATH_EXECUTOR] Emergency duration: " << g_emergencyManager->getEmergencyDuration() << "s" << std::endl;
        return;
    }
    
    std::cout << "[PATH_EXECUTOR] Current state: ";
    switch (currentState) {
        case ExecutionState::IDLE: std::cout << "IDLE"; break;
        case ExecutionState::EXECUTING: std::cout << "EXECUTING"; break;
        case ExecutionState::PAUSED: std::cout << "PAUSED"; break;
        case ExecutionState::COMPLETED: std::cout << "COMPLETED"; break;
        case ExecutionState::ERROR: std::cout << "ERROR"; break;
        case ExecutionState::EMERGENCY_STOP: std::cout << "EMERGENCY_STOP"; break;
    }
    std::cout << std::endl;
    
    std::cout << "[PATH_EXECUTOR] Path length: " << executionPath.size() << " waypoints" << std::endl;
    std::cout << "[PATH_EXECUTOR] Current waypoint index: " << currentWaypointIndex << std::endl;
    std::cout << "[PATH_EXECUTOR] Execution progress: " << getExecutionProgress() * 100.0 << "%" << std::endl;
    
    if (currentState == ExecutionState::EXECUTING) {
        std::cout << "[PATH_EXECUTOR] Execution time: " << getExecutionTime().count() << "s" << std::endl;
    }
    
    std::cout << "[PATH_EXECUTOR] Path execution validation completed" << std::endl;
}
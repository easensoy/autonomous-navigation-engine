#include "environment/EnvironmentManager.hpp"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <mutex>

class EnvironmentSetter {
private:
    EnvironmentManager* environmentManager;
    mutable std::mutex environmentMutex;
    std::chrono::steady_clock::time_point lastSetTime;
    std::string previousEnvironmentName;
    bool transitionInProgress;
    
    struct EnvironmentTransition {
        std::shared_ptr<Graph> sourceEnvironment;
        std::shared_ptr<Graph> targetEnvironment;
        std::string sourceName;
        std::string targetName;
        std::chrono::steady_clock::time_point startTime;
        double transitionDuration;
        bool successful;
    };
    
    std::vector<EnvironmentTransition> transitionHistory;
    size_t maxHistorySize;
    
public:
    explicit EnvironmentSetter(EnvironmentManager* envManager) 
        : environmentManager(envManager), transitionInProgress(false), maxHistorySize(10) {
        
        if (!environmentManager) {
            throw std::invalid_argument("EnvironmentManager pointer cannot be null");
        }
        
        lastSetTime = std::chrono::steady_clock::now();
    }
    
    bool setEnvironmentSafely(std::shared_ptr<Graph> newEnvironment, const std::string& name) {
        std::lock_guard<std::mutex> lock(environmentMutex);
        
        std::cout << "[SET_ENVIRONMENT] Initiating safe environment transition to: " << name << std::endl;
        
        if (transitionInProgress) {
            std::cout << "[SET_ENVIRONMENT] Environment transition already in progress" << std::endl;
            return false;
        }
        
        transitionInProgress = true;
        auto transitionStart = std::chrono::steady_clock::now();
        
        try {
            // Validate new environment before setting
            if (!validateEnvironmentForSetting(newEnvironment, name)) {
                transitionInProgress = false;
                return false;
            }
            
            // Store current environment for potential rollback
            auto currentEnvironment = environmentManager->getCurrentEnvironment();
            previousEnvironmentName = environmentManager->getEnvironmentName();
            
            // Perform the environment transition
            bool success = performEnvironmentTransition(currentEnvironment, newEnvironment, name);
            
            // Record transition in history
            recordTransition(currentEnvironment, newEnvironment, previousEnvironmentName, name, transitionStart, success);
            
            if (success) {
                lastSetTime = std::chrono::steady_clock::now();
                std::cout << "[SET_ENVIRONMENT] Environment transition completed successfully" << std::endl;
            } else {
                std::cout << "[SET_ENVIRONMENT] Environment transition failed" << std::endl;
            }
            
            transitionInProgress = false;
            return success;
            
        } catch (const std::exception& e) {
            std::cout << "[SET_ENVIRONMENT] Exception during environment transition: " << e.what() << std::endl;
            transitionInProgress = false;
            return false;
        }
    }
    
    bool setEnvironmentWithValidation(std::shared_ptr<Graph> newEnvironment, const std::string& name) {
        std::cout << "[SET_ENVIRONMENT] Setting environment with comprehensive validation: " << name << std::endl;
        
        if (!newEnvironment) {
            std::cout << "[SET_ENVIRONMENT] Cannot set null environment" << std::endl;
            return false;
        }
        
        // Perform pre-setting validation
        if (!performPreSettingValidation(newEnvironment, name)) {
            return false;
        }
        
        // Execute the setting with monitoring
        bool success = setEnvironmentSafely(newEnvironment, name);
        
        if (success) {
            // Perform post-setting validation
            if (!performPostSettingValidation(name)) {
                std::cout << "[SET_ENVIRONMENT] Post-setting validation failed, attempting rollback" << std::endl;
                rollbackToPreviousEnvironment();
                return false;
            }
        }
        
        return success;
    }
    
    bool rollbackToPreviousEnvironment() {
        std::lock_guard<std::mutex> lock(environmentMutex);
        
        std::cout << "[SET_ENVIRONMENT] Attempting rollback to previous environment" << std::endl;
        
        if (transitionHistory.empty()) {
            std::cout << "[SET_ENVIRONMENT] No previous environment available for rollback" << std::endl;
            return false;
        }
        
        const EnvironmentTransition& lastTransition = transitionHistory.back();
        
        if (!lastTransition.sourceEnvironment) {
            std::cout << "[SET_ENVIRONMENT] Previous environment is no longer available" << std::endl;
            return false;
        }
        
        try {
            environmentManager->setEnvironment(lastTransition.sourceEnvironment, lastTransition.sourceName);
            
            std::cout << "[SET_ENVIRONMENT] Successfully rolled back to environment: " 
                      << lastTransition.sourceName << std::endl;
            
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "[SET_ENVIRONMENT] Rollback failed: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool setEnvironmentFromFile(const std::string& filePath) {
        std::cout << "[SET_ENVIRONMENT] Loading and setting environment from file: " << filePath << std::endl;
        
        try {
            // Create temporary environment manager to load the file
            EnvironmentManager tempManager;
            
            if (!tempManager.loadEnvironment(filePath)) {
                std::cout << "[SET_ENVIRONMENT] Failed to load environment from file" << std::endl;
                return false;
            }
            
            auto loadedEnvironment = tempManager.getCurrentEnvironment();
            if (!loadedEnvironment) {
                std::cout << "[SET_ENVIRONMENT] Loaded environment is null" << std::endl;
                return false;
            }
            
            // Extract environment name from file path
            std::string environmentName = extractEnvironmentNameFromPath(filePath);
            
            return setEnvironmentWithValidation(loadedEnvironment, environmentName);
            
        } catch (const std::exception& e) {
            std::cout << "[SET_ENVIRONMENT] Exception while loading from file: " << e.what() << std::endl;
            return false;
        }
    }
    
    void configureEnvironmentSettings(const std::unordered_map<std::string, std::string>& settings) {
        std::cout << "[SET_ENVIRONMENT] Configuring environment settings" << std::endl;
        
        for (const auto& [key, value] : settings) {
            std::cout << "[SET_ENVIRONMENT] Setting configuration: " << key << " = " << value << std::endl;
            
            if (key == "validation_level") {
                configureValidationLevel(value);
            } else if (key == "transition_timeout") {
                configureTransitionTimeout(std::stod(value));
            } else if (key == "auto_backup") {
                configureAutoBackup(value == "true");
            } else {
                std::cout << "[SET_ENVIRONMENT] Unknown configuration key: " << key << std::endl;
            }
        }
    }
    
    bool isTransitionInProgress() const {
        std::lock_guard<std::mutex> lock(environmentMutex);
        return transitionInProgress;
    }
    
    std::chrono::duration<double> getTimeSinceLastSet() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - lastSetTime);
    }
    
    void generateTransitionReport() const {
        std::cout << "\n[SET_ENVIRONMENT] === ENVIRONMENT TRANSITION REPORT ===" << std::endl;
        
        std::cout << "[SET_ENVIRONMENT] Current environment: " 
                  << environmentManager->getEnvironmentName() << std::endl;
        std::cout << "[SET_ENVIRONMENT] Time since last transition: " 
                  << getTimeSinceLastSet().count() << " seconds" << std::endl;
        std::cout << "[SET_ENVIRONMENT] Transition in progress: " 
                  << (transitionInProgress ? "Yes" : "No") << std::endl;
        
        std::cout << "[SET_ENVIRONMENT] Transition history (" << transitionHistory.size() << " entries):" << std::endl;
        
        for (size_t i = 0; i < transitionHistory.size(); ++i) {
            const auto& transition = transitionHistory[i];
            std::cout << "[SET_ENVIRONMENT]   [" << i << "] " << transition.sourceName 
                      << " -> " << transition.targetName 
                      << " (" << transition.transitionDuration << "s, " 
                      << (transition.successful ? "Success" : "Failed") << ")" << std::endl;
        }
        
        std::cout << "[SET_ENVIRONMENT] === END REPORT ===" << std::endl;
    }
    
private:
    bool validateEnvironmentForSetting(std::shared_ptr<Graph> environment, const std::string& name) {
        std::cout << "[SET_ENVIRONMENT] Validating environment for setting: " << name << std::endl;
        
        if (!environment) {
            std::cout << "[SET_ENVIRONMENT] Environment is null" << std::endl;
            return false;
        }
        
        if (name.empty()) {
            std::cout << "[SET_ENVIRONMENT] Environment name is empty" << std::endl;
            return false;
        }
        
        if (environment->isEmpty()) {
            std::cout << "[SET_ENVIRONMENT] Environment is empty" << std::endl;
            return false;
        }
        
        // Check minimum complexity requirements
        if (environment->getNodeCount() < 2) {
            std::cout << "[SET_ENVIRONMENT] Environment has insufficient nodes (minimum 2 required)" << std::endl;
            return false;
        }
        
        // Validate connectivity
        if (!environment->isConnected()) {
            std::cout << "[SET_ENVIRONMENT] Warning: Environment is not fully connected" << std::endl;
            // Don't fail on this, just warn
        }
        
        std::cout << "[SET_ENVIRONMENT] Environment validation passed" << std::endl;
        return true;
    }
    
    bool performEnvironmentTransition(std::shared_ptr<Graph> currentEnv, 
                                    std::shared_ptr<Graph> newEnv, 
                                    const std::string& name) {
        std::cout << "[SET_ENVIRONMENT] Performing environment transition to: " << name << std::endl;
        
        try {
            // Pre-transition backup if auto-backup is enabled
            if (currentEnv) {
                environmentManager->backupCurrentEnvironment();
            }
            
            // Set the new environment
            environmentManager->setEnvironment(newEnv, name);
            
            // Validate the transition was successful
            if (!environmentManager->validateCurrentEnvironment()) {
                std::cout << "[SET_ENVIRONMENT] Environment validation failed after setting" << std::endl;
                return false;
            }
            
            // Verify the environment was set correctly
            if (environmentManager->getEnvironmentName() != name) {
                std::cout << "[SET_ENVIRONMENT] Environment name mismatch after setting" << std::endl;
                return false;
            }
            
            auto setEnvironment = environmentManager->getCurrentEnvironment();
            if (!setEnvironment || setEnvironment != newEnv) {
                std::cout << "[SET_ENVIRONMENT] Environment pointer mismatch after setting" << std::endl;
                return false;
            }
            
            std::cout << "[SET_ENVIRONMENT] Environment transition completed successfully" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "[SET_ENVIRONMENT] Exception during transition: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool performPreSettingValidation(std::shared_ptr<Graph> environment, const std::string& name) {
        std::cout << "[SET_ENVIRONMENT] Performing pre-setting validation" << std::endl;
        
        // Check for name conflicts
        if (name == environmentManager->getEnvironmentName()) {
            std::cout << "[SET_ENVIRONMENT] Warning: Setting environment with same name as current" << std::endl;
        }
        
        // Memory usage check
        size_t estimatedMemoryUsage = estimateEnvironmentMemoryUsage(environment);
        if (estimatedMemoryUsage > 100 * 1024 * 1024) { // 100MB limit
            std::cout << "[SET_ENVIRONMENT] Warning: Environment may use significant memory (" 
                      << estimatedMemoryUsage / (1024 * 1024) << " MB)" << std::endl;
        }
        
        // Structural integrity check
        if (!checkEnvironmentStructuralIntegrity(environment)) {
            std::cout << "[SET_ENVIRONMENT] Environment structural integrity check failed" << std::endl;
            return false;
        }
        
        std::cout << "[SET_ENVIRONMENT] Pre-setting validation completed" << std::endl;
        return true;
    }
    
    bool performPostSettingValidation(const std::string& expectedName) {
        std::cout << "[SET_ENVIRONMENT] Performing post-setting validation" << std::endl;
        
        // Verify environment manager state
        if (!environmentManager->isEnvironmentSet()) {
            std::cout << "[SET_ENVIRONMENT] Environment manager reports no environment set" << std::endl;
            return false;
        }
        
        if (environmentManager->getEnvironmentName() != expectedName) {
            std::cout << "[SET_ENVIRONMENT] Environment name mismatch in post-validation" << std::endl;
            return false;
        }
        
        if (!environmentManager->validateCurrentEnvironment()) {
            std::cout << "[SET_ENVIRONMENT] Environment validation failed in post-validation" << std::endl;
            return false;
        }
        
        std::cout << "[SET_ENVIRONMENT] Post-setting validation completed successfully" << std::endl;
        return true;
    }
    
    void recordTransition(std::shared_ptr<Graph> sourceEnv, std::shared_ptr<Graph> targetEnv,
                         const std::string& sourceName, const std::string& targetName,
                         std::chrono::steady_clock::time_point startTime, bool successful) {
        auto endTime = std::chrono::steady_clock::now();
        double duration = std::chrono::duration<double>(endTime - startTime).count();
        
        EnvironmentTransition transition;
        transition.sourceEnvironment = sourceEnv;
        transition.targetEnvironment = targetEnv;
        transition.sourceName = sourceName;
        transition.targetName = targetName;
        transition.startTime = startTime;
        transition.transitionDuration = duration;
        transition.successful = successful;
        
        transitionHistory.push_back(transition);
        
        // Limit history size
        if (transitionHistory.size() > maxHistorySize) {
            transitionHistory.erase(transitionHistory.begin());
        }
        
        std::cout << "[SET_ENVIRONMENT] Transition recorded: " << sourceName << " -> " << targetName 
                  << " (" << duration << "s, " << (successful ? "Success" : "Failed") << ")" << std::endl;
    }
    
    std::string extractEnvironmentNameFromPath(const std::string& filePath) const {
        size_t lastSlash = filePath.find_last_of("/\\");
        size_t lastDot = filePath.find_last_of(".");
        
        if (lastSlash == std::string::npos) {
            lastSlash = 0;
        } else {
            lastSlash++;
        }
        
        if (lastDot == std::string::npos || lastDot < lastSlash) {
            return filePath.substr(lastSlash);
        }
        
        return filePath.substr(lastSlash, lastDot - lastSlash);
    }
    
    size_t estimateEnvironmentMemoryUsage(std::shared_ptr<Graph> environment) const {
        if (!environment) {
            return 0;
        }
        
        size_t nodeMemory = environment->getNodeCount() * sizeof(Node);
        size_t edgeMemory = environment->getEdgeCount() * sizeof(Edge);
        size_t overhead = 1024; // Estimated overhead
        
        return nodeMemory + edgeMemory + overhead;
    }
    
    bool checkEnvironmentStructuralIntegrity(std::shared_ptr<Graph> environment) const {
        if (!environment) {
            return false;
        }
        
        try {
            // Check that all edges reference valid nodes
            for (int nodeId : environment->getAllNodeIds()) {
                const std::vector<Edge>& edges = environment->getEdgesFrom(nodeId);
                
                for (const Edge& edge : edges) {
                    if (!environment->hasNode(edge.getFromNode()) || 
                        !environment->hasNode(edge.getToNode())) {
                        std::cout << "[SET_ENVIRONMENT] Invalid edge references non-existent node" << std::endl;
                        return false;
                    }
                    
                    if (edge.getWeight() < 0) {
                        std::cout << "[SET_ENVIRONMENT] Warning: Negative edge weight detected" << std::endl;
                    }
                }
            }
            
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "[SET_ENVIRONMENT] Exception during structural integrity check: " << e.what() << std::endl;
            return false;
        }
    }
    
    void configureValidationLevel(const std::string& level) {
        std::cout << "[SET_ENVIRONMENT] Setting validation level to: " << level << std::endl;
        // Implementation would set internal validation flags
    }
    
    void configureTransitionTimeout(double timeoutSeconds) {
        std::cout << "[SET_ENVIRONMENT] Setting transition timeout to: " << timeoutSeconds << " seconds" << std::endl;
        // Implementation would set timeout for transitions
    }
    
    void configureAutoBackup(bool enabled) {
        std::cout << "[SET_ENVIRONMENT] Auto backup " << (enabled ? "enabled" : "disabled") << std::endl;
        // Implementation would configure automatic backup behavior
    }
};

// Global environment setter instance
static std::unique_ptr<EnvironmentSetter> g_environmentSetter;

// Implementation methods for EnvironmentManager
void EnvironmentManager::setEnvironment(std::shared_ptr<Graph> newEnvironment) {
    setEnvironment(newEnvironment, "UnnamedEnvironment");
}

void EnvironmentManager::setEnvironment(std::shared_ptr<Graph> newEnvironment, const std::string& name) {
    std::cout << "[ENV_MANAGER] Setting environment: " << name << std::endl;
    
    if (!g_environmentSetter) {
        g_environmentSetter = std::make_unique<EnvironmentSetter>(this);
    }
    
    if (!g_environmentSetter->setEnvironmentWithValidation(newEnvironment, name)) {
        throw std::runtime_error("Failed to set environment: " + name);
    }
    
    // Update internal state
    currentEnvironment = newEnvironment;
    environmentName = name;
    isEnvironmentValid = true;
    
    // Add to history
    environmentHistory.push_back(newEnvironment);
    if (environmentHistory.size() > 5) {
        environmentHistory.erase(environmentHistory.begin());
    }
    
    logEnvironmentChange("", name);
    validateEnvironmentIntegrity();
}

bool EnvironmentManager::trySetEnvironment(std::shared_ptr<Graph> newEnvironment) {
    std::cout << "[ENV_MANAGER] Attempting to set environment safely" << std::endl;
    
    try {
        if (!g_environmentSetter) {
            g_environmentSetter = std::make_unique<EnvironmentSetter>(this);
        }
        
        bool success = g_environmentSetter->setEnvironmentSafely(newEnvironment, "TrialEnvironment");
        
        if (success) {
            currentEnvironment = newEnvironment;
            environmentName = "TrialEnvironment";
            isEnvironmentValid = true;
            
            environmentHistory.push_back(newEnvironment);
            if (environmentHistory.size() > 5) {
                environmentHistory.erase(environmentHistory.begin());
            }
        }
        
        return success;
        
    } catch (const std::exception& e) {
        std::cout << "[ENV_MANAGER] Exception in trySetEnvironment: " << e.what() << std::endl;
        return false;
    }
}
#include "environment/EnvironmentManager.hpp"
#include <iostream>
#include <unordered_set>
#include <algorithm>

class EnvironmentSwitchingManager {
private:
    EnvironmentManager* environmentManager;
    std::vector<std::shared_ptr<Graph>> environmentCache;
    std::vector<std::string> environmentNames;
    std::unordered_map<std::string, size_t> nameToIndexMap;
    size_t currentEnvironmentIndex;
    size_t maxCacheSize;
    bool preloadingEnabled;
    
    struct SwitchingMetrics {
        std::chrono::steady_clock::time_point lastSwitchTime;
        double averageSwitchTime;
        size_t switchCount;
        std::vector<double> switchTimes;
        size_t failedSwitches;
    };
    
    SwitchingMetrics metrics;
    
public:
    explicit EnvironmentSwitchingManager(EnvironmentManager* envManager) 
        : environmentManager(envManager), currentEnvironmentIndex(0), 
          maxCacheSize(5), preloadingEnabled(true) {
        
        initializeMetrics();
    }
    
    bool switchToEnvironment(const std::string& environmentName) {
        std::cout << "[ENV_SWITCHING] Switching to environment: " << environmentName << std::endl;
        
        auto switchStart = std::chrono::steady_clock::now();
        
        // Check if environment is already cached
        auto it = nameToIndexMap.find(environmentName);
        if (it != nameToIndexMap.end()) {
            return performCachedSwitch(it->second, switchStart);
        }
        
        // Load environment from file if not cached
        return performFileBasedSwitch(environmentName, switchStart);
    }
    
    bool switchToEnvironmentByIndex(size_t environmentIndex) {
        std::cout << "[ENV_SWITCHING] Switching to environment by index: " << environmentIndex << std::endl;
        
        if (environmentIndex >= environmentCache.size()) {
            std::cout << "[ENV_SWITCHING] Invalid environment index: " << environmentIndex << std::endl;
            metrics.failedSwitches++;
            return false;
        }
        
        auto switchStart = std::chrono::steady_clock::now();
        return performCachedSwitch(environmentIndex, switchStart);
    }
    
    void preloadEnvironment(const std::string& environmentPath) {
        std::cout << "[ENV_SWITCHING] Preloading environment: " << environmentPath << std::endl;
        
        if (!preloadingEnabled) {
            std::cout << "[ENV_SWITCHING] Preloading is disabled" << std::endl;
            return;
        }
        
        if (environmentCache.size() >= maxCacheSize) {
            evictOldestEnvironment();
        }
        
        try {
            // Create temporary environment manager for loading
            EnvironmentManager tempManager;
            if (tempManager.loadEnvironment(environmentPath)) {
                auto loadedEnvironment = tempManager.getCurrentEnvironment();
                
                if (loadedEnvironment) {
                    // Extract environment name from path
                    std::string envName = extractEnvironmentName(environmentPath);
                    
                    // Add to cache
                    environmentCache.push_back(loadedEnvironment);
                    environmentNames.push_back(envName);
                    nameToIndexMap[envName] = environmentCache.size() - 1;
                    
                    std::cout << "[ENV_SWITCHING] Environment preloaded successfully: " << envName << std::endl;
                } else {
                    std::cout << "[ENV_SWITCHING] Failed to get loaded environment" << std::endl;
                }
            } else {
                std::cout << "[ENV_SWITCHING] Failed to load environment from: " << environmentPath << std::endl;
            }
        } catch (const std::exception& e) {
            std::cout << "[ENV_SWITCHING] Exception during preloading: " << e.what() << std::endl;
        }
    }
    
    void preloadMultipleEnvironments(const std::vector<std::string>& environmentPaths) {
        std::cout << "[ENV_SWITCHING] Preloading " << environmentPaths.size() << " environments" << std::endl;
        
        for (const std::string& path : environmentPaths) {
            preloadEnvironment(path);
        }
        
        std::cout << "[ENV_SWITCHING] Preloading completed. Cache size: " << environmentCache.size() << std::endl;
    }
    
    std::vector<std::string> listCachedEnvironments() const {
        return environmentNames;
    }
    
    bool validateEnvironmentCompatibility(const std::string& fromEnvironment, const std::string& toEnvironment) {
        std::cout << "[ENV_SWITCHING] Validating compatibility: " << fromEnvironment << " -> " << toEnvironment << std::endl;
        
        auto fromIt = nameToIndexMap.find(fromEnvironment);
        auto toIt = nameToIndexMap.find(toEnvironment);
        
        if (fromIt == nameToIndexMap.end() || toIt == nameToIndexMap.end()) {
            std::cout << "[ENV_SWITCHING] One or both environments not found in cache" << std::endl;
            return false;
        }
        
        auto fromEnv = environmentCache[fromIt->second];
        auto toEnv = environmentCache[toIt->second];
        
        // Check structural compatibility
        if (!checkStructuralCompatibility(fromEnv.get(), toEnv.get())) {
            std::cout << "[ENV_SWITCHING] Structural compatibility check failed" << std::endl;
            return false;
        }
        
        // Check node ID overlap
        if (!checkNodeCompatibility(fromEnv.get(), toEnv.get())) {
            std::cout << "[ENV_SWITCHING] Node compatibility check failed" << std::endl;
            return false;
        }
        
        std::cout << "[ENV_SWITCHING] Environment compatibility validated successfully" << std::endl;
        return true;
    }
    
    void clearEnvironmentCache() {
        std::cout << "[ENV_SWITCHING] Clearing environment cache" << std::endl;
        
        environmentCache.clear();
        environmentNames.clear();
        nameToIndexMap.clear();
        currentEnvironmentIndex = 0;
        
        std::cout << "[ENV_SWITCHING] Environment cache cleared" << std::endl;
    }
    
    void setMaxCacheSize(size_t maxSize) {
        maxCacheSize = maxSize;
        std::cout << "[ENV_SWITCHING] Maximum cache size set to: " << maxSize << std::endl;
        
        // Evict environments if cache is now too large
        while (environmentCache.size() > maxCacheSize) {
            evictOldestEnvironment();
        }
    }
    
    void enablePreloading(bool enable) {
        preloadingEnabled = enable;
        std::cout << "[ENV_SWITCHING] Preloading " << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    size_t getCurrentEnvironmentIndex() const {
        return currentEnvironmentIndex;
    }
    
    std::string getCurrentEnvironmentName() const {
        if (currentEnvironmentIndex < environmentNames.size()) {
            return environmentNames[currentEnvironmentIndex];
        }
        return "";
    }
    
    double getAverageSwitchTime() const {
        return metrics.averageSwitchTime;
    }
    
    size_t getSwitchCount() const {
        return metrics.switchCount;
    }
    
    size_t getFailedSwitchCount() const {
        return metrics.failedSwitches;
    }
    
    void generateSwitchingReport() const {
        std::cout << "\n[ENV_SWITCHING] === ENVIRONMENT SWITCHING REPORT ===" << std::endl;
        
        std::cout << "[ENV_SWITCHING] Cache Status:" << std::endl;
        std::cout << "[ENV_SWITCHING]   Cached environments: " << environmentCache.size() << std::endl;
        std::cout << "[ENV_SWITCHING]   Maximum cache size: " << maxCacheSize << std::endl;
        std::cout << "[ENV_SWITCHING]   Current environment: " << getCurrentEnvironmentName() << std::endl;
        std::cout << "[ENV_SWITCHING]   Preloading enabled: " << (preloadingEnabled ? "Yes" : "No") << std::endl;
        
        std::cout << "[ENV_SWITCHING] Performance Metrics:" << std::endl;
        std::cout << "[ENV_SWITCHING]   Total switches: " << metrics.switchCount << std::endl;
        std::cout << "[ENV_SWITCHING]   Failed switches: " << metrics.failedSwitches << std::endl;
        std::cout << "[ENV_SWITCHING]   Average switch time: " << metrics.averageSwitchTime << " seconds" << std::endl;
        
        if (!metrics.switchTimes.empty()) {
            auto minMax = std::minmax_element(metrics.switchTimes.begin(), metrics.switchTimes.end());
            std::cout << "[ENV_SWITCHING]   Fastest switch: " << *minMax.first << " seconds" << std::endl;
            std::cout << "[ENV_SWITCHING]   Slowest switch: " << *minMax.second << " seconds" << std::endl;
        }
        
        std::cout << "[ENV_SWITCHING] Cached Environments:" << std::endl;
        for (size_t i = 0; i < environmentNames.size(); ++i) {
            std::cout << "[ENV_SWITCHING]   [" << i << "] " << environmentNames[i];
            if (i == currentEnvironmentIndex) {
                std::cout << " (current)";
            }
            std::cout << std::endl;
        }
        
        std::cout << "[ENV_SWITCHING] === END REPORT ===" << std::endl;
    }
    
private:
    void initializeMetrics() {
        metrics.lastSwitchTime = std::chrono::steady_clock::now();
        metrics.averageSwitchTime = 0.0;
        metrics.switchCount = 0;
        metrics.failedSwitches = 0;
        metrics.switchTimes.clear();
    }
    
    bool performCachedSwitch(size_t environmentIndex, std::chrono::steady_clock::time_point switchStart) {
        std::cout << "[ENV_SWITCHING] Performing cached environment switch to index: " << environmentIndex << std::endl;
        
        try {
            auto targetEnvironment = environmentCache[environmentIndex];
            std::string targetName = environmentNames[environmentIndex];
            
            // Validate environment before switching
            if (!validateEnvironmentIntegrity(targetEnvironment.get())) {
                std::cout << "[ENV_SWITCHING] Target environment failed integrity check" << std::endl;
                metrics.failedSwitches++;
                return false;
            }
            
            // Perform the switch
            environmentManager->setEnvironment(targetEnvironment, targetName);
            currentEnvironmentIndex = environmentIndex;
            
            // Update metrics
            updateSwitchingMetrics(switchStart);
            
            std::cout << "[ENV_SWITCHING] Successfully switched to cached environment: " << targetName << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "[ENV_SWITCHING] Exception during cached switch: " << e.what() << std::endl;
            metrics.failedSwitches++;
            return false;
        }
    }
    
    bool performFileBasedSwitch(const std::string& environmentName, std::chrono::steady_clock::time_point switchStart) {
        std::cout << "[ENV_SWITCHING] Performing file-based environment switch: " << environmentName << std::endl;
        
        try {
            // Attempt to load environment
            if (!environmentManager->loadEnvironment(environmentName)) {
                std::cout << "[ENV_SWITCHING] Failed to load environment: " << environmentName << std::endl;
                metrics.failedSwitches++;
                return false;
            }
            
            // Add to cache if space available
            if (environmentCache.size() < maxCacheSize) {
                addCurrentEnvironmentToCache(environmentName);
            }
            
            // Update metrics
            updateSwitchingMetrics(switchStart);
            
            std::cout << "[ENV_SWITCHING] Successfully switched to file-based environment: " << environmentName << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "[ENV_SWITCHING] Exception during file-based switch: " << e.what() << std::endl;
            metrics.failedSwitches++;
            return false;
        }
    }
    
    void addCurrentEnvironmentToCache(const std::string& environmentName) {
        auto currentEnvironment = environmentManager->getCurrentEnvironment();
        if (currentEnvironment) {
            environmentCache.push_back(currentEnvironment);
            environmentNames.push_back(environmentName);
            nameToIndexMap[environmentName] = environmentCache.size() - 1;
            currentEnvironmentIndex = environmentCache.size() - 1;
            
            std::cout << "[ENV_SWITCHING] Added environment to cache: " << environmentName << std::endl;
        }
    }
    
    void evictOldestEnvironment() {
        if (environmentCache.empty()) {
            return;
        }
        
        std::cout << "[ENV_SWITCHING] Evicting oldest environment from cache" << std::endl;
        
        // Remove the first (oldest) environment
        std::string evictedName = environmentNames[0];
        
        environmentCache.erase(environmentCache.begin());
        environmentNames.erase(environmentNames.begin());
        
        // Update name-to-index mapping
        nameToIndexMap.erase(evictedName);
        for (size_t i = 0; i < environmentNames.size(); ++i) {
            nameToIndexMap[environmentNames[i]] = i;
        }
        
        // Adjust current environment index
        if (currentEnvironmentIndex > 0) {
            currentEnvironmentIndex--;
        }
        
        std::cout << "[ENV_SWITCHING] Evicted environment: " << evictedName << std::endl;
    }
    
    bool checkStructuralCompatibility(const Graph* fromEnv, const Graph* toEnv) {
        // Check if environments have compatible structure for switching
        
        // Check size compatibility (shouldn't be vastly different)
        double sizeRatio = static_cast<double>(toEnv->getNodeCount()) / fromEnv->getNodeCount();
        if (sizeRatio > 10.0 || sizeRatio < 0.1) {
            std::cout << "[ENV_SWITCHING] Environments have incompatible sizes" << std::endl;
            return false;
        }
        
        // Check connectivity patterns
        double fromDensity = calculateGraphDensity(fromEnv);
        double toDensity = calculateGraphDensity(toEnv);
        
        if (std::abs(fromDensity - toDensity) > 0.5) {
            std::cout << "[ENV_SWITCHING] Environments have significantly different connectivity patterns" << std::endl;
            return false;
        }
        
        return true;
    }
    
    bool checkNodeCompatibility(const Graph* fromEnv, const Graph* toEnv) {
        // Check for potential node ID conflicts or compatibility issues
        
        std::vector<int> fromNodes = fromEnv->getAllNodeIds();
        std::vector<int> toNodes = toEnv->getAllNodeIds();
        
        std::unordered_set<int> fromNodeSet(fromNodes.begin(), fromNodes.end());
        std::unordered_set<int> toNodeSet(toNodes.begin(), toNodes.end());
        
        // Check for common nodes (which might indicate related environments)
        size_t commonNodes = 0;
        for (int nodeId : fromNodes) {
            if (toNodeSet.find(nodeId) != toNodeSet.end()) {
                commonNodes++;
            }
        }
        
        double overlapRatio = static_cast<double>(commonNodes) / std::min(fromNodes.size(), toNodes.size());
        
        // If there's partial overlap, environments might be related but incompatible for direct switching
        if (overlapRatio > 0.1 && overlapRatio < 0.8) {
            std::cout << "[ENV_SWITCHING] Environments have problematic node overlap: " << overlapRatio << std::endl;
            return false;
        }
        
        return true;
    }
    
    bool validateEnvironmentIntegrity(const Graph* environment) {
        // Basic integrity checks
        if (!environment || environment->isEmpty()) {
            return false;
        }
        
        if (environment->getNodeCount() == 0) {
            return false;
        }
        
        // Check if environment is connected (basic validation)
        if (!environment->isConnected()) {
            std::cout << "[ENV_SWITCHING] Warning: Target environment is not fully connected" << std::endl;
            // Don't fail on this, just warn
        }
        
        return true;
    }
    
    double calculateGraphDensity(const Graph* graph) {
        size_t nodeCount = graph->getNodeCount();
        if (nodeCount <= 1) {
            return 0.0;
        }
        
        size_t edgeCount = graph->getEdgeCount();
        size_t maxPossibleEdges = nodeCount * (nodeCount - 1) / 2;
        
        return static_cast<double>(edgeCount) / maxPossibleEdges;
    }
    
    void updateSwitchingMetrics(std::chrono::steady_clock::time_point switchStart) {
        auto switchEnd = std::chrono::steady_clock::now();
        double switchTime = std::chrono::duration<double>(switchEnd - switchStart).count();
        
        metrics.switchTimes.push_back(switchTime);
        metrics.switchCount++;
        metrics.lastSwitchTime = switchEnd;
        
        // Update average switch time
        double totalTime = std::accumulate(metrics.switchTimes.begin(), metrics.switchTimes.end(), 0.0);
        metrics.averageSwitchTime = totalTime / metrics.switchTimes.size();
        
        // Limit stored switch times to avoid memory growth
        if (metrics.switchTimes.size() > 100) {
            metrics.switchTimes.erase(metrics.switchTimes.begin());
        }
        
        std::cout << "[ENV_SWITCHING] Switch completed in " << switchTime << " seconds" << std::endl;
    }
    
    std::string extractEnvironmentName(const std::string& environmentPath) {
        size_t lastSlash = environmentPath.find_last_of("/\\");
        size_t lastDot = environmentPath.find_last_of(".");
        
        if (lastSlash == std::string::npos) {
            lastSlash = 0;
        } else {
            lastSlash++;
        }
        
        if (lastDot == std::string::npos || lastDot < lastSlash) {
            return environmentPath.substr(lastSlash);
        }
        
        return environmentPath.substr(lastSlash, lastDot - lastSlash);
    }
};
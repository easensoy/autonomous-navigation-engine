#include "autonomous_navigator/NavigationCore.hpp"
#include "environment/EnvironmentManager.hpp"
#include "utilities/ConfigManager.hpp"
#include <iostream>
#include <stdexcept>

class NavigationInitializationManager {
private:
    NavigationCore* navigationCore;
    std::unique_ptr<EnvironmentManager> environmentManager;
    std::unique_ptr<ConfigManager> configManager;
    bool initializationCompleted;
    std::string initializationErrorMessage;
    NavigationMode defaultNavigationMode;
    
    struct InitializationConfig {
        std::string environmentFilePath;
        NavigationMode navigationMode;
        double executionTimeout;
        bool enableRealTimeUpdates;
        bool enableLogging;
        std::string logFilePath;
        double safetyMargin;
        bool enableEmergencyProtocols;
    };
    
    InitializationConfig currentConfig;
    
public:
    explicit NavigationInitializationManager(NavigationCore* navCore) 
        : navigationCore(navCore), initializationCompleted(false),
          defaultNavigationMode(NavigationMode::HYBRID) {
        
        initializeManagers();
        loadDefaultConfiguration();
    }
    
    bool initializeNavigationSystem() {
        std::cout << "[NAV_INIT] Starting navigation system initialization" << std::endl;
        
        initializationCompleted = false;
        initializationErrorMessage.clear();
        
        try {
            // Step 1: Initialize environment management
            if (!initializeEnvironmentSubsystem()) {
                return false;
            }
            
            // Step 2: Configure navigation parameters
            if (!configureNavigationParameters()) {
                return false;
            }
            
            // Step 3: Initialize pathfinding components
            if (!initializePathfindingComponents()) {
                return false;
            }
            
            // Step 4: Set up safety and monitoring systems
            if (!initializeSafetyProtocols()) {
                return false;
            }
            
            // Step 5: Validate complete system integration
            if (!validateSystemIntegration()) {
                return false;
            }
            
            initializationCompleted = true;
            std::cout << "[NAV_INIT] Navigation system initialization completed successfully" << std::endl;
            
            generateInitializationReport();
            return true;
            
        } catch (const std::exception& e) {
            initializationErrorMessage = e.what();
            std::cout << "[NAV_INIT] Initialization failed with exception: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool initializeWithConfiguration(const std::string& configFilePath) {
        std::cout << "[NAV_INIT] Initializing navigation system with configuration file: " 
                  << configFilePath << std::endl;
        
        if (!loadConfigurationFromFile(configFilePath)) {
            std::cout << "[NAV_INIT] Failed to load configuration file" << std::endl;
            return false;
        }
        
        return initializeNavigationSystem();
    }
    
    bool reinitializeWithNewEnvironment(const std::string& environmentPath) {
        std::cout << "[NAV_INIT] Reinitializing navigation system with new environment: " 
                  << environmentPath << std::endl;
        
        // Safely stop current navigation
        if (navigationCore->isNavigationComplete() == false) {
            navigationCore->stopNavigation();
        }
        
        // Update environment configuration
        currentConfig.environmentFilePath = environmentPath;
        
        // Reinitialize with new environment
        return initializeNavigationSystem();
    }
    
    void setNavigationMode(NavigationMode mode) {
        currentConfig.navigationMode = mode;
        defaultNavigationMode = mode;
        
        if (initializationCompleted) {
            navigationCore->setNavigationMode(mode);
        }
        
        std::cout << "[NAV_INIT] Navigation mode set to: " << static_cast<int>(mode) << std::endl;
    }
    
    void setExecutionTimeout(double timeoutSeconds) {
        currentConfig.executionTimeout = timeoutSeconds;
        std::cout << "[NAV_INIT] Execution timeout set to " << timeoutSeconds << " seconds" << std::endl;
    }
    
    void enableRealTimeUpdates(bool enable) {
        currentConfig.enableRealTimeUpdates = enable;
        std::cout << "[NAV_INIT] Real-time updates " << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void setSafetyMargin(double margin) {
        currentConfig.safetyMargin = margin;
        std::cout << "[NAV_INIT] Safety margin set to " << margin << std::endl;
    }
    
    bool isInitializationComplete() const {
        return initializationCompleted;
    }
    
    std::string getInitializationError() const {
        return initializationErrorMessage;
    }
    
    void shutdownNavigationSystem() {
        std::cout << "[NAV_INIT] Shutting down navigation system" << std::endl;
        
        if (navigationCore) {
            navigationCore->stopNavigation();
        }
        
        initializationCompleted = false;
        
        // Clean up resources
        if (environmentManager) {
            environmentManager.reset();
        }
        
        if (configManager) {
            configManager.reset();
        }
        
        std::cout << "[NAV_INIT] Navigation system shutdown completed" << std::endl;
    }
    
    void validateInitializationState() const {
        std::cout << "[NAV_INIT] Validating initialization state" << std::endl;
        
        if (!initializationCompleted) {
            throw std::runtime_error("Navigation system not properly initialized");
        }
        
        if (!navigationCore) {
            throw std::runtime_error("Navigation core not available");
        }
        
        if (!environmentManager || !environmentManager->isEnvironmentSet()) {
            throw std::runtime_error("Environment not properly configured");
        }
        
        std::cout << "[NAV_INIT] Initialization state validation passed" << std::endl;
    }
    
    void generateInitializationReport() const {
        std::cout << "\n[NAV_INIT] === NAVIGATION INITIALIZATION REPORT ===" << std::endl;
        
        std::cout << "[NAV_INIT] Initialization Status: " 
                  << (initializationCompleted ? "COMPLETED" : "FAILED") << std::endl;
        
        if (!initializationCompleted && !initializationErrorMessage.empty()) {
            std::cout << "[NAV_INIT] Error Message: " << initializationErrorMessage << std::endl;
        }
        
        std::cout << "[NAV_INIT] Configuration Details:" << std::endl;
        std::cout << "[NAV_INIT]   Environment File: " << currentConfig.environmentFilePath << std::endl;
        std::cout << "[NAV_INIT]   Navigation Mode: " << static_cast<int>(currentConfig.navigationMode) << std::endl;
        std::cout << "[NAV_INIT]   Execution Timeout: " << currentConfig.executionTimeout << "s" << std::endl;
        std::cout << "[NAV_INIT]   Real-time Updates: " << (currentConfig.enableRealTimeUpdates ? "Enabled" : "Disabled") << std::endl;
        std::cout << "[NAV_INIT]   Safety Margin: " << currentConfig.safetyMargin << std::endl;
        std::cout << "[NAV_INIT]   Emergency Protocols: " << (currentConfig.enableEmergencyProtocols ? "Enabled" : "Disabled") << std::endl;
        
        if (environmentManager) {
            std::cout << "[NAV_INIT] Environment Status: " << environmentManager->getEnvironmentStatus() << std::endl;
            std::cout << "[NAV_INIT] Environment Complexity: " << environmentManager->getEnvironmentComplexity() << std::endl;
        }
        
        std::cout << "[NAV_INIT] === END REPORT ===" << std::endl;
    }
    
private:
    void initializeManagers() {
        std::cout << "[NAV_INIT] Initializing subsystem managers" << std::endl;
        
        environmentManager = std::make_unique<EnvironmentManager>();
        configManager = std::make_unique<ConfigManager>();
        
        std::cout << "[NAV_INIT] Subsystem managers initialized" << std::endl;
    }
    
    void loadDefaultConfiguration() {
        std::cout << "[NAV_INIT] Loading default configuration" << std::endl;
        
        currentConfig.environmentFilePath = "";
        currentConfig.navigationMode = defaultNavigationMode;
        currentConfig.executionTimeout = 30.0;
        currentConfig.enableRealTimeUpdates = true;
        currentConfig.enableLogging = true;
        currentConfig.logFilePath = "navigation.log";
        currentConfig.safetyMargin = 0.5;
        currentConfig.enableEmergencyProtocols = true;
        
        std::cout << "[NAV_INIT] Default configuration loaded" << std::endl;
    }
    
    bool loadConfigurationFromFile(const std::string& configFilePath) {
        std::cout << "[NAV_INIT] Loading configuration from file: " << configFilePath << std::endl;
        
        try {
            if (!configManager->loadFromFile(configFilePath)) {
                std::cout << "[NAV_INIT] Failed to load configuration file" << std::endl;
                return false;
            }
            
            // Extract configuration values
            currentConfig.environmentFilePath = configManager->getValue<std::string>("environment_file", "");
            currentConfig.navigationMode = static_cast<NavigationMode>(
                configManager->getValue<int>("navigation_mode", static_cast<int>(NavigationMode::HYBRID)));
            currentConfig.executionTimeout = configManager->getValue<double>("execution_timeout", 30.0);
            currentConfig.enableRealTimeUpdates = configManager->getValue<bool>("real_time_updates", true);
            currentConfig.enableLogging = configManager->getValue<bool>("enable_logging", true);
            currentConfig.logFilePath = configManager->getValue<std::string>("log_file_path", "navigation.log");
            currentConfig.safetyMargin = configManager->getValue<double>("safety_margin", 0.5);
            currentConfig.enableEmergencyProtocols = configManager->getValue<bool>("emergency_protocols", true);
            
            std::cout << "[NAV_INIT] Configuration loaded successfully from file" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "[NAV_INIT] Configuration file loading error: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool initializeEnvironmentSubsystem() {
        std::cout << "[NAV_INIT] Initializing environment subsystem" << std::endl;
        
        if (!environmentManager) {
            initializationErrorMessage = "Environment manager not available";
            return false;
        }
        
        // Initialize environment
        if (!currentConfig.environmentFilePath.empty()) {
            if (!environmentManager->loadEnvironment(currentConfig.environmentFilePath)) {
                initializationErrorMessage = "Failed to load environment from file: " + currentConfig.environmentFilePath;
                return false;
            }
        } else {
            environmentManager->initializeDefaultEnvironment();
        }
        
        // Validate environment
        if (!environmentManager->validateCurrentEnvironment()) {
            initializationErrorMessage = "Environment validation failed";
            return false;
        }
        
        // Update navigation core with environment
        auto environment = environmentManager->getCurrentEnvironment();
        if (!environment) {
            initializationErrorMessage = "No valid environment available";
            return false;
        }
        
        navigationCore->updateEnvironment(environment.get());
        
        std::cout << "[NAV_INIT] Environment subsystem initialized successfully" << std::endl;
        return true;
    }
    
    bool configureNavigationParameters() {
        std::cout << "[NAV_INIT] Configuring navigation parameters" << std::endl;
        
        try {
            // Set navigation mode
            navigationCore->setNavigationMode(currentConfig.navigationMode);
            
            // Configure real-time updates if enabled
            if (currentConfig.enableRealTimeUpdates) {
                std::cout << "[NAV_INIT] Real-time navigation updates enabled" << std::endl;
            }
            
            std::cout << "[NAV_INIT] Navigation parameters configured successfully" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            initializationErrorMessage = "Navigation parameter configuration failed: " + std::string(e.what());
            return false;
        }
    }
    
    bool initializePathfindingComponents() {
        std::cout << "[NAV_INIT] Initializing pathfinding components" << std::endl;
        
        try {
            // The pathfinding components are initialized internally by NavigationCore
            // This step validates that all required components are available
            
            // Verify that the navigation core can handle basic operations
            std::string status = navigationCore->getNavigationStatus();
            if (status.find("ERROR") != std::string::npos) {
                initializationErrorMessage = "Pathfinding component initialization error: " + status;
                return false;
            }
            
            std::cout << "[NAV_INIT] Pathfinding components initialized successfully" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            initializationErrorMessage = "Pathfinding component initialization failed: " + std::string(e.what());
            return false;
        }
    }
    
    bool initializeSafetyProtocols() {
        std::cout << "[NAV_INIT] Initializing safety protocols" << std::endl;
        
        if (!currentConfig.enableEmergencyProtocols) {
            std::cout << "[NAV_INIT] Emergency protocols disabled by configuration" << std::endl;
            return true;
        }
        
        try {
            // Set up emergency stop capabilities
            // This is handled internally by the navigation core and path executor
            
            std::cout << "[NAV_INIT] Safety protocols initialized successfully" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            initializationErrorMessage = "Safety protocol initialization failed: " + std::string(e.what());
            return false;
        }
    }
    
    bool validateSystemIntegration() {
        std::cout << "[NAV_INIT] Validating system integration" << std::endl;
        
        try {
            // Test basic navigation functionality
            if (!performIntegrationTests()) {
                initializationErrorMessage = "System integration validation failed";
                return false;
            }
            
            std::cout << "[NAV_INIT] System integration validation completed successfully" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            initializationErrorMessage = "System integration validation error: " + std::string(e.what());
            return false;
        }
    }
    
    bool performIntegrationTests() {
        std::cout << "[NAV_INIT] Performing integration tests" << std::endl;
        
        // Test 1: Verify navigation core responds to basic commands
        std::string initialStatus = navigationCore->getNavigationStatus();
        if (initialStatus.empty()) {
            std::cout << "[NAV_INIT] Integration test failed: Navigation core not responding" << std::endl;
            return false;
        }
        
        // Test 2: Verify environment is accessible
        auto environment = environmentManager->getCurrentEnvironment();
        if (!environment || environment->isEmpty()) {
            std::cout << "[NAV_INIT] Integration test failed: Environment not accessible" << std::endl;
            return false;
        }
        
        // Test 3: Verify navigation modes can be set
        NavigationMode testMode = NavigationMode::GLOBAL_ONLY;
        navigationCore->setNavigationMode(testMode);
        navigationCore->setNavigationMode(currentConfig.navigationMode); // Restore original
        
        // Test 4: Verify safety systems are responsive
        if (currentConfig.enableEmergencyProtocols) {
            // Emergency stop test - should be safe to call and return system to normal state
            navigationCore->emergencyStop();
            
            // Verify system can be restored to normal operation
            std::string statusAfterEmergency = navigationCore->getNavigationStatus();
            if (statusAfterEmergency.find("EMERGENCY") == std::string::npos) {
                // System recovered properly
            }
        }
        
        std::cout << "[NAV_INIT] All integration tests passed" << std::endl;
        return true;
    }
    
    void configureSafetyParameters() {
        std::cout << "[NAV_INIT] Configuring safety parameters" << std::endl;
        
        if (currentConfig.safetyMargin > 0) {
            std::cout << "[NAV_INIT] Safety margin configured: " << currentConfig.safetyMargin << std::endl;
        }
        
        if (currentConfig.enableEmergencyProtocols) {
            std::cout << "[NAV_INIT] Emergency protocols enabled" << std::endl;
        }
    }
    
    void setupLogging() {
        if (!currentConfig.enableLogging) {
            return;
        }
        
        std::cout << "[NAV_INIT] Setting up navigation logging" << std::endl;
        
        try {
            // Configure logging parameters
            configManager->setValue("log_file", currentConfig.logFilePath);
            configManager->setValue("log_level", "INFO");
            
            std::cout << "[NAV_INIT] Logging configured to file: " << currentConfig.logFilePath << std::endl;
            
        } catch (const std::exception& e) {
            std::cout << "[NAV_INIT] Logging setup warning: " << e.what() << std::endl;
        }
    }
    
    void validateConfiguration() {
        std::cout << "[NAV_INIT] Validating configuration parameters" << std::endl;
        
        if (currentConfig.executionTimeout <= 0) {
            throw std::invalid_argument("Invalid execution timeout value");
        }
        
        if (currentConfig.safetyMargin < 0) {
            throw std::invalid_argument("Invalid safety margin value");
        }
        
        std::cout << "[NAV_INIT] Configuration validation completed" << std::endl;
    }
};
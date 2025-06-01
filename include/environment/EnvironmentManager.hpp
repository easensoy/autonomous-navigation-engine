#pragma once
#include "../core/Graph.hpp"
#include <memory>
#include <string>
#include <vector>

class EnvironmentManager {
private:
    std::shared_ptr<Graph> currentEnvironment;
    std::string environmentName;
    bool isEnvironmentValid;
    std::vector<std::shared_ptr<Graph>> environmentHistory;
    
    void validateEnvironmentIntegrity();
    void logEnvironmentChange(const std::string& oldName, const std::string& newName);

public:
    EnvironmentManager();
    explicit EnvironmentManager(std::shared_ptr<Graph> environment);
    ~EnvironmentManager() = default;
    
    void setEnvironment(std::shared_ptr<Graph> newEnvironment);
    void setEnvironment(std::shared_ptr<Graph> newEnvironment, const std::string& name);
    bool trySetEnvironment(std::shared_ptr<Graph> newEnvironment);
    
    bool validateCurrentEnvironment() const;
    bool isEnvironmentSet() const;
    std::string getEnvironmentStatus() const;
    
    std::shared_ptr<Graph> getCurrentEnvironment() const;
    const std::string& getEnvironmentName() const;
    size_t getEnvironmentComplexity() const;
    
    void initializeDefaultEnvironment();
    void initializeFromConfiguration(const std::string& configFile);
    void createEmptyEnvironment(const std::string& name);
    
    void saveEnvironment(const std::string& filename) const;
    bool loadEnvironment(const std::string& filename);
    void backupCurrentEnvironment();
    bool restorePreviousEnvironment();
};
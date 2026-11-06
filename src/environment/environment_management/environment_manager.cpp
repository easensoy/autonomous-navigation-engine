#include "environment/EnvironmentManager.hpp"
#include <iostream>
#include <memory>
#include <string>
#include <utility>

// EnvironmentManager construction, queries and validation. Switching, persistence and backups
// are implemented in set_environment.cpp and environment_persistence.cpp.

// Defined in set_environment.cpp and environment_persistence.cpp.
void releaseEnvironmentSetter(const EnvironmentManager* manager);
void releaseEnvironmentPersistence(const EnvironmentManager* manager);

EnvironmentManager::EnvironmentManager()
    : currentEnvironment(nullptr), environmentName("None"), isEnvironmentValid(false) {}

EnvironmentManager::EnvironmentManager(std::shared_ptr<Graph> environment)
    : currentEnvironment(std::move(environment)), environmentName("Default"), isEnvironmentValid(false) {
    if (currentEnvironment) {
        environmentHistory.push_back(currentEnvironment);
    }
    validateEnvironmentIntegrity();
}

EnvironmentManager::~EnvironmentManager() {
    releaseEnvironmentSetter(this);
    releaseEnvironmentPersistence(this);
}

void EnvironmentManager::applyEnvironment(std::shared_ptr<Graph> environment, const std::string& name) {
    const std::string previous = environmentName;
    currentEnvironment = std::move(environment);
    environmentName = name;
    environmentHistory.push_back(currentEnvironment);
    if (environmentHistory.size() > 5) {
        environmentHistory.erase(environmentHistory.begin());
    }
    logEnvironmentChange(previous, name);
    validateEnvironmentIntegrity();
}

void EnvironmentManager::validateEnvironmentIntegrity() {
    isEnvironmentValid = validateCurrentEnvironment();
}

void EnvironmentManager::logEnvironmentChange(const std::string& oldName, const std::string& newName) {
    std::cout << "[ENV_MANAGER] Environment changed: " << (oldName.empty() ? std::string("(none)") : oldName)
              << " -> " << newName << std::endl;
}

bool EnvironmentManager::validateCurrentEnvironment() const {
    if (!currentEnvironment || currentEnvironment->isEmpty()) {
        return false;
    }
    // Every edge must end at a node that exists.
    for (int nodeId : currentEnvironment->getAllNodeIds()) {
        for (const Edge& edge : currentEnvironment->getEdgesFrom(nodeId)) {
            if (!currentEnvironment->hasNode(edge.getToNode()) || edge.getWeight() != edge.getWeight()) {
                return false;  // dangling edge or NaN weight
            }
        }
    }
    return true;
}

bool EnvironmentManager::isEnvironmentSet() const {
    return currentEnvironment != nullptr;
}

std::string EnvironmentManager::getEnvironmentStatus() const {
    if (!currentEnvironment) {
        return "No environment set";
    }
    return "Environment '" + environmentName + "': " + std::to_string(currentEnvironment->getNodeCount()) +
           " nodes, " + std::to_string(currentEnvironment->getEdgeCount()) + " edges, " +
           (validateCurrentEnvironment() ? "valid" : "invalid");
}

std::shared_ptr<Graph> EnvironmentManager::getCurrentEnvironment() const {
    return currentEnvironment;
}

const std::string& EnvironmentManager::getEnvironmentName() const {
    return environmentName;
}

size_t EnvironmentManager::getEnvironmentComplexity() const {
    if (!currentEnvironment) {
        return 0;
    }
    return currentEnvironment->getNodeCount() + currentEnvironment->getEdgeCount();
}

void EnvironmentManager::initializeDefaultEnvironment() {
    // A 5x5 grid, 10 units apart, 4-connected with weight 10 (the same layout src/main.cpp uses).
    auto grid = std::make_shared<Graph>();
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            grid->addNode(i * 5 + j, "Node_" + std::to_string(i * 5 + j), i * 10.0, j * 10.0);
        }
    }
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            const int id = i * 5 + j;
            if (j < 4) grid->addEdge(id, id + 1, 10.0);
            if (i < 4) grid->addEdge(id, id + 5, 10.0);
        }
    }
    applyEnvironment(grid, "DefaultGrid");
}

void EnvironmentManager::initializeFromConfiguration(const std::string& configFile) {
    if (!loadEnvironment(configFile)) {
        std::cout << "[ENV_MANAGER] Could not load " << configFile << ", using the default environment" << std::endl;
        initializeDefaultEnvironment();
    }
}

void EnvironmentManager::createEmptyEnvironment(const std::string& name) {
    applyEnvironment(std::make_shared<Graph>(), name);  // an empty graph never validates
}

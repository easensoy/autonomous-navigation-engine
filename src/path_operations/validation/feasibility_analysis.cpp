#include "path_operations/PathValidator.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <unordered_set>

class FeasibilityAnalyzer {
private:
    const Graph* graph;
    
    struct FeasibilityMetrics {
        bool isExecutable;
        bool hasAlternatives;
        double executionProbability;
        double resourceRequirement;
        double timeComplexity;
        double riskLevel;
        std::vector<std::string> feasibilityIssues;
        std::vector<int> criticalNodes;
        std::vector<std::pair<int, int>> problematicEdges;
        
        FeasibilityMetrics() : isExecutable(true), hasAlternatives(false), 
                              executionProbability(1.0), resourceRequirement(1.0),
                              timeComplexity(1.0), riskLevel(0.0) {}
    };
    
    struct EnvironmentalConstraints {
        std::unordered_map<int, double> nodeAccessibility;
        std::unordered_map<std::string, double> edgeReliability;
        std::unordered_set<int> temporarilyBlockedNodes;
        std::unordered_set<std::string> maintenanceEdges;
        double globalAccessibilityFactor;
        
        EnvironmentalConstraints() : globalAccessibilityFactor(1.0) {}
    };
    
    EnvironmentalConstraints constraints;
    bool enableRealTimeChecks;
    double feasibilityThreshold;
    double resourceCapacity;
    
public:
    FeasibilityAnalyzer(const Graph* environment) 
        : graph(environment), enableRealTimeChecks(true), 
          feasibilityThreshold(0.7), resourceCapacity(100.0) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[FEASIBILITY] Feasibility analyzer initialized" << std::endl;
    }
    
    FeasibilityMetrics analyzeFeasibility(const std::vector<int>& path) {
        FeasibilityMetrics metrics;
        
        if (path.size() < 2) {
            metrics.isExecutable = false;
            metrics.feasibilityIssues.push_back("Path too short for execution");
            return metrics;
        }
        
        std::cout << "[FEASIBILITY] Analyzing feasibility for path with " << path.size() << " nodes" << std::endl;
        
        // Check basic executability
        metrics.isExecutable = checkBasicExecutability(path, metrics);
        
        // Calculate execution probability
        metrics.executionProbability = calculateExecutionProbability(path, metrics);
        
        // Assess resource requirements
        metrics.resourceRequirement = calculateResourceRequirement(path, metrics);
        
        // Analyze time complexity
        metrics.timeComplexity = calculateTimeComplexity(path, metrics);
        
        // Evaluate risk level
        metrics.riskLevel = calculateRiskLevel(path, metrics);
        
        // Check for alternatives
        metrics.hasAlternatives = checkAlternativeAvailability(path, metrics);
        
        // Perform real-time checks if enabled
        if (enableRealTimeChecks) {
            performRealTimeChecks(path, metrics);
        }
        
        // Final feasibility determination
        determineFinalFeasibility(metrics);
        
        std::cout << "[FEASIBILITY] Analysis complete - Execution probability: " 
                  << metrics.executionProbability << ", Risk level: " << metrics.riskLevel << std::endl;
        
        return metrics;
    }
    
private:
    bool checkBasicExecutability(const std::vector<int>& path, FeasibilityMetrics& metrics) {
        bool executable = true;
        
        // Check node existence and accessibility
        for (size_t i = 0; i < path.size(); ++i) {
            if (!graph->hasNode(path[i])) {
                metrics.feasibilityIssues.push_back("Node " + std::to_string(path[i]) + " does not exist");
                executable = false;
                continue;
            }
            
            // Check if node is temporarily blocked
            if (constraints.temporarilyBlockedNodes.find(path[i]) != constraints.temporarilyBlockedNodes.end()) {
                metrics.feasibilityIssues.push_back("Node " + std::to_string(path[i]) + " is temporarily blocked");
                metrics.criticalNodes.push_back(path[i]);
                executable = false;
            }
            
            // Check node accessibility
            auto accessIt = constraints.nodeAccessibility.find(path[i]);
            if (accessIt != constraints.nodeAccessibility.end() && accessIt->second < 0.5) {
                metrics.feasibilityIssues.push_back("Node " + std::to_string(path[i]) + " has low accessibility");
                metrics.criticalNodes.push_back(path[i]);
            }
        }
        
        // Check edge connectivity and reliability
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasNode(path[i-1]) || !graph->hasNode(path[i])) {
                continue; // Skip if nodes don't exist
            }
            
            if (!graph->hasEdge(path[i-1], path[i])) {
                metrics.feasibilityIssues.push_back("No edge between nodes " + 
                    std::to_string(path[i-1]) + " and " + std::to_string(path[i]));
                metrics.problematicEdges.emplace_back(path[i-1], path[i]);
                executable = false;
                continue;
            }
            
            // Check edge reliability
            std::string edgeKey = std::to_string(path[i-1]) + "_" + std::to_string(path[i]);
            auto reliabilityIt = constraints.edgeReliability.find(edgeKey);
            if (reliabilityIt != constraints.edgeReliability.end() && reliabilityIt->second < 0.6) {
                metrics.feasibilityIssues.push_back("Edge " + edgeKey + " has low reliability");
                metrics.problematicEdges.emplace_back(path[i-1], path[i]);
            }
            
            // Check for maintenance edges
            if (constraints.maintenanceEdges.find(edgeKey) != constraints.maintenanceEdges.end()) {
                metrics.feasibilityIssues.push_back("Edge " + edgeKey + " is under maintenance");
                metrics.problematicEdges.emplace_back(path[i-1], path[i]);
                executable = false;
            }
        }
        
        return executable;
    }
    
    double calculateExecutionProbability(const std::vector<int>& path, FeasibilityMetrics& metrics) {
        double probability = 1.0;
        
        // Factor in node accessibility
        for (int nodeId : path) {
            auto accessIt = constraints.nodeAccessibility.find(nodeId);
            if (accessIt != constraints.nodeAccessibility.end()) {
                probability *= accessIt->second;
            }
        }
        
        // Factor in edge reliability
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasNode(path[i-1]) || !graph->hasNode(path[i])) continue;
            
            std::string edgeKey = std::to_string(path[i-1]) + "_" + std::to_string(path[i]);
            auto reliabilityIt = constraints.edgeReliability.find(edgeKey);
            if (reliabilityIt != constraints.edgeReliability.end()) {
                probability *= reliabilityIt->second;
            } else {
                probability *= 0.95; // Default reliability
            }
        }
        
        // Apply global accessibility factor
        probability *= constraints.globalAccessibilityFactor;
        
        // Path length penalty (longer paths have lower probability)
        double lengthPenalty = std::exp(-static_cast<double>(path.size()) / 50.0);
        probability *= lengthPenalty;
        
        return std::max(0.0, std::min(1.0, probability));
    }
    
    double calculateResourceRequirement(const std::vector<int>& path, FeasibilityMetrics& metrics) {
        double totalRequirement = 0.0;
        
        // Base resource requirement per node
        totalRequirement += path.size() * 1.0;
        
        // Additional requirement for problematic nodes
        for (int nodeId : metrics.criticalNodes) {
            totalRequirement += 5.0; // Extra resources for critical nodes
        }
        
        // Additional requirement for problematic edges
        for (const auto& edge : metrics.problematicEdges) {
            totalRequirement += 3.0; // Extra resources for problematic edges
        }
        
        // Calculate path distance as resource factor
        double totalDistance = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            if (graph->hasNode(path[i-1]) && graph->hasNode(path[i])) {
                const Node& from = graph->getNode(path[i-1]);
                const Node& to = graph->getNode(path[i]);
                totalDistance += from.euclideanDistance(to);
            }
        }
        totalRequirement += totalDistance * 0.1;
        
        return totalRequirement;
    }
    
    double calculateTimeComplexity(const std::vector<int>& path, FeasibilityMetrics& metrics) {
        double baseTime = static_cast<double>(path.size());
        
        // Add delay for critical nodes
        double criticalNodeDelay = metrics.criticalNodes.size() * 2.0;
        
        // Add delay for problematic edges
        double problematicEdgeDelay = metrics.problematicEdges.size() * 1.5;
        
        // Add delay based on path complexity (turns, backtracking)
        double complexityDelay = calculatePathComplexity(path) * 0.5;
        
        return baseTime + criticalNodeDelay + problematicEdgeDelay + complexityDelay;
    }
    
    double calculateRiskLevel(const std::vector<int>& path, FeasibilityMetrics& metrics) {
        double risk = 0.0;
        
        // Risk from critical nodes
        risk += metrics.criticalNodes.size() * 0.2;
        
        // Risk from problematic edges
        risk += metrics.problematicEdges.size() * 0.15;
        
        // Risk from low execution probability
        if (metrics.executionProbability < 0.8) {
            risk += (0.8 - metrics.executionProbability) * 0.5;
        }
        
        // Risk from high resource requirement
        if (metrics.resourceRequirement > resourceCapacity * 0.8) {
            risk += 0.3;
        }
        
        // Environmental risk factors
        risk += (1.0 - constraints.globalAccessibilityFactor) * 0.2;
        
        return std::max(0.0, std::min(1.0, risk));
    }
    
    bool checkAlternativeAvailability(const std::vector<int>& path, FeasibilityMetrics& metrics) {
        if (path.size() < 2) return false;
        
        int startNode = path.front();
        int endNode = path.back();
        
        // Simple check: count available neighbors at start and end
        std::vector<int> startNeighbors = graph->getNeighbors(startNode);
        std::vector<int> endNeighbors = graph->getNeighbors(endNode);
        
        // Remove blocked neighbors
        auto isBlocked = [this](int nodeId) {
            return constraints.temporarilyBlockedNodes.find(nodeId) != 
                   constraints.temporarilyBlockedNodes.end();
        };
        
        startNeighbors.erase(std::remove_if(startNeighbors.begin(), startNeighbors.end(), isBlocked),
                            startNeighbors.end());
        endNeighbors.erase(std::remove_if(endNeighbors.begin(), endNeighbors.end(), isBlocked),
                          endNeighbors.end());
        
        // Alternative exists if there are multiple ways to start or end the journey
        return (startNeighbors.size() > 1 || endNeighbors.size() > 1);
    }
    
    void performRealTimeChecks(const std::vector<int>& path, FeasibilityMetrics& metrics) {
        auto currentTime = std::chrono::steady_clock::now();
        
        // Simulate real-time constraint updates
        updateTemporaryBlocks();
        updateAccessibilityFactors();
        updateMaintenanceStatus();
        
        // Re-check critical aspects that might have changed
        for (int nodeId : path) {
            if (constraints.temporarilyBlockedNodes.find(nodeId) != constraints.temporarilyBlockedNodes.end()) {
                if (std::find(metrics.criticalNodes.begin(), metrics.criticalNodes.end(), nodeId) == 
                    metrics.criticalNodes.end()) {
                    metrics.criticalNodes.push_back(nodeId);
                    metrics.feasibilityIssues.push_back("Real-time: Node " + std::to_string(nodeId) + " newly blocked");
                }
            }
        }
    }
    
    void determineFinalFeasibility(FeasibilityMetrics& metrics) {
        // Override executability based on overall assessment
        if (metrics.executionProbability < feasibilityThreshold) {
            metrics.isExecutable = false;
            metrics.feasibilityIssues.push_back("Execution probability below threshold");
        }
        
        if (metrics.resourceRequirement > resourceCapacity) {
            metrics.isExecutable = false;
            metrics.feasibilityIssues.push_back("Resource requirement exceeds capacity");
        }
        
        if (metrics.riskLevel > 0.8) {
            metrics.isExecutable = false;
            metrics.feasibilityIssues.push_back("Risk level too high for safe execution");
        }
    }
    
    double calculatePathComplexity(const std::vector<int>& path) {
        if (path.size() < 3) return 0.0;
        
        double complexity = 0.0;
        
        // Count direction changes
        for (size_t i = 2; i < path.size(); ++i) {
            if (!graph->hasNode(path[i-2]) || !graph->hasNode(path[i-1]) || !graph->hasNode(path[i])) {
                continue;
            }
            
            const Node& prev = graph->getNode(path[i-2]);
            const Node& curr = graph->getNode(path[i-1]);
            const Node& next = graph->getNode(path[i]);
            
            // Calculate angle
            double v1x = curr.getX() - prev.getX();
            double v1y = curr.getY() - prev.getY();
            double v2x = next.getX() - curr.getX();
            double v2y = next.getY() - curr.getY();
            
            double dot = v1x * v2x + v1y * v2y;
            double mag1 = std::sqrt(v1x * v1x + v1y * v1y);
            double mag2 = std::sqrt(v2x * v2x + v2y * v2y);
            
            if (mag1 > 0 && mag2 > 0) {
                double cosAngle = dot / (mag1 * mag2);
                double angle = std::acos(std::max(-1.0, std::min(1.0, cosAngle)));
                complexity += (M_PI - angle) / M_PI; // Normalize angle change
            }
        }
        
        return complexity;
    }
    
    void updateTemporaryBlocks() {
        // Simulate dynamic blocking (would integrate with real systems)
        // For now, this is a placeholder
    }
    
    void updateAccessibilityFactors() {
        // Simulate accessibility changes
        // For now, this is a placeholder
    }
    
    void updateMaintenanceStatus() {
        // Simulate maintenance schedule updates
        // For now, this is a placeholder
    }
    
public:
    void setNodeAccessibility(int nodeId, double accessibility) {
        constraints.nodeAccessibility[nodeId] = std::max(0.0, std::min(1.0, accessibility));
        std::cout << "[FEASIBILITY] Set accessibility for node " << nodeId << " to " << accessibility << std::endl;
    }
    
    void setEdgeReliability(int fromId, int toId, double reliability) {
        std::string edgeKey = std::to_string(fromId) + "_" + std::to_string(toId);
        constraints.edgeReliability[edgeKey] = std::max(0.0, std::min(1.0, reliability));
        std::cout << "[FEASIBILITY] Set reliability for edge " << edgeKey << " to " << reliability << std::endl;
    }
    
    void addTemporaryBlock(int nodeId) {
        constraints.temporarilyBlockedNodes.insert(nodeId);
        std::cout << "[FEASIBILITY] Added temporary block for node " << nodeId << std::endl;
    }
    
    void removeTemporaryBlock(int nodeId) {
        constraints.temporarilyBlockedNodes.erase(nodeId);
        std::cout << "[FEASIBILITY] Removed temporary block for node " << nodeId << std::endl;
    }
    
    void addMaintenanceEdge(int fromId, int toId) {
        std::string edgeKey = std::to_string(fromId) + "_" + std::to_string(toId);
        constraints.maintenanceEdges.insert(edgeKey);
        std::cout << "[FEASIBILITY] Added maintenance status for edge " << edgeKey << std::endl;
    }
    
    void setGlobalAccessibilityFactor(double factor) {
        constraints.globalAccessibilityFactor = std::max(0.0, std::min(1.0, factor));
        std::cout << "[FEASIBILITY] Set global accessibility factor to " << factor << std::endl;
    }
    
    void setFeasibilityThreshold(double threshold) {
        feasibilityThreshold = std::max(0.0, std::min(1.0, threshold));
        std::cout << "[FEASIBILITY] Set feasibility threshold to " << threshold << std::endl;
    }
    
    void setResourceCapacity(double capacity) {
        resourceCapacity = capacity;
        std::cout << "[FEASIBILITY] Set resource capacity to " << capacity << std::endl;
    }
    
    void enableRealTime(bool enable) {
        enableRealTimeChecks = enable;
        std::cout << "[FEASIBILITY] Real-time checks " << (enable ? "enabled" : "disabled") << std::endl;
    }
};

// Global feasibility analyzer
static std::unique_ptr<FeasibilityAnalyzer> g_feasibilityAnalyzer;

bool analyzePathFeasibility(const Graph* graph, const std::vector<int>& path, 
                           double& executionProbability, std::string& feasibilityReport) {
    if (!g_feasibilityAnalyzer) {
        g_feasibilityAnalyzer = std::make_unique<FeasibilityAnalyzer>(graph);
    }
    
    auto metrics = g_feasibilityAnalyzer->analyzeFeasibility(path);
    executionProbability = metrics.executionProbability;
    
    // Generate feasibility report
    feasibilityReport = "Feasibility Analysis Report:\n";
    feasibilityReport += "Executable: " + std::string(metrics.isExecutable ? "Yes" : "No") + "\n";
    feasibilityReport += "Execution Probability: " + std::to_string(metrics.executionProbability) + "\n";
    feasibilityReport += "Resource Requirement: " + std::to_string(metrics.resourceRequirement) + "\n";
    feasibilityReport += "Risk Level: " + std::to_string(metrics.riskLevel) + "\n";
    feasibilityReport += "Has Alternatives: " + std::string(metrics.hasAlternatives ? "Yes" : "No") + "\n";
    
    if (!metrics.feasibilityIssues.empty()) {
        feasibilityReport += "Issues:\n";
        for (const auto& issue : metrics.feasibilityIssues) {
            feasibilityReport += "- " + issue + "\n";
        }
    }
    
    return metrics.isExecutable;
}

void configureFeasibilityAnalysis(double threshold, double capacity, bool realTimeEnabled) {
    if (g_feasibilityAnalyzer) {
        g_feasibilityAnalyzer->setFeasibilityThreshold(threshold);
        g_feasibilityAnalyzer->setResourceCapacity(capacity);
        g_feasibilityAnalyzer->enableRealTime(realTimeEnabled);
    }
}
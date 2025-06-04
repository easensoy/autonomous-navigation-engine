#include "path_operations/PathValidator.hpp"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <chrono>

// Forward declarations for the validation modules
bool verifyPathConstraints(const Graph* graph, const std::vector<int>& path, 
                          std::string& violationReport, bool strictMode);
bool analyzePathFeasibility(const Graph* graph, const std::vector<int>& path, 
                           double& executionProbability, std::string& feasibilityReport);
bool checkPathSafety(const Graph* graph, const std::vector<int>& path, 
                    double& safetyScore, std::string& safetyReport);

PathValidator::PathValidator(const Graph* graph) 
    : environment(graph), strictValidation(false), maxAllowedSegmentLength(50.0) {
    
    if (!environment) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
    
    std::cout << "[PATH_VALIDATOR] Path validator initialized" << std::endl;
}

PathValidator::PathValidator(const Graph* graph, bool strict) 
    : environment(graph), strictValidation(strict), maxAllowedSegmentLength(50.0) {
    
    if (!environment) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
    
    std::cout << "[PATH_VALIDATOR] Path validator initialized with strict mode " 
              << (strict ? "enabled" : "disabled") << std::endl;
}

bool PathValidator::validatePath(const std::vector<int>& path) const {
    if (path.empty()) {
        std::cout << "[PATH_VALIDATOR] Validation failed: empty path" << std::endl;
        return false;
    }
    
    std::cout << "[PATH_VALIDATOR] Validating path with " << path.size() << " nodes" << std::endl;
    
    // Quick structural validation
    if (!validatePathStructure(path)) {
        std::cout << "[PATH_VALIDATOR] Structural validation failed" << std::endl;
        return false;
    }
    
    // Check connectivity
    if (!validatePathConnectivity(path)) {
        std::cout << "[PATH_VALIDATOR] Connectivity validation failed" << std::endl;
        return false;
    }
    
    // Check safety if strict validation is enabled
    if (strictValidation && !validatePathSafety(path)) {
        std::cout << "[PATH_VALIDATOR] Safety validation failed" << std::endl;
        return false;
    }
    
    std::cout << "[PATH_VALIDATOR] Path validation successful" << std::endl;
    return true;
}

bool PathValidator::validatePathWithDetails(const std::vector<int>& path, std::string& errorMessage) const {
    errorMessage.clear();
    
    if (path.empty()) {
        errorMessage = "Path is empty";
        return false;
    }
    
    std::cout << "[PATH_VALIDATOR] Detailed validation for path with " << path.size() << " nodes" << std::endl;
    
    std::ostringstream detailedReport;
    bool isValid = true;
    
    // Constraint verification
    std::string constraintReport;
    if (!verifyPathConstraints(environment, path, constraintReport, strictValidation)) {
        detailedReport << "Constraint Violations:\n" << constraintReport << "\n";
        isValid = false;
    }
    
    // Feasibility analysis
    double executionProbability;
    std::string feasibilityReport;
    if (!analyzePathFeasibility(environment, path, executionProbability, feasibilityReport)) {
        detailedReport << "Feasibility Issues:\n" << feasibilityReport << "\n";
        isValid = false;
    }
    
    // Safety checking
    double safetyScore;
    std::string safetyReport;
    if (!checkPathSafety(environment, path, safetyScore, safetyReport)) {
        detailedReport << "Safety Concerns:\n" << safetyReport << "\n";
        isValid = false;
    }
    
    errorMessage = detailedReport.str();
    
    if (isValid) {
        std::cout << "[PATH_VALIDATOR] Detailed validation successful" << std::endl;
    } else {
        std::cout << "[PATH_VALIDATOR] Detailed validation failed with issues" << std::endl;
    }
    
    return isValid;
}

bool PathValidator::isPathSafe(const std::vector<int>& path) const {
    double safetyScore;
    std::string safetyReport;
    return checkPathSafety(environment, path, safetyScore, safetyReport);
}

bool PathValidator::checkPathExists(const std::vector<int>& path) const {
    if (path.empty()) return false;
    
    // Check all nodes exist
    for (int nodeId : path) {
        if (!environment->hasNode(nodeId)) {
            std::cout << "[PATH_VALIDATOR] Node " << nodeId << " does not exist" << std::endl;
            return false;
        }
    }
    
    return true;
}

bool PathValidator::checkPathContinuity(const std::vector<int>& path) const {
    if (path.size() < 2) return true;
    
    // Check all consecutive nodes are connected
    for (size_t i = 1; i < path.size(); ++i) {
        if (!environment->hasEdge(path[i-1], path[i])) {
            std::cout << "[PATH_VALIDATOR] No edge between nodes " << path[i-1] 
                      << " and " << path[i] << std::endl;
            return false;
        }
    }
    
    return true;
}

bool PathValidator::checkPathEfficiency(const std::vector<int>& path) const {
    if (path.size() < 3) return true;
    
    double pathLength = calculatePathLength(path);
    double directLength = calculateDirectDistance(path.front(), path.back());
    
    // Path is inefficient if it's more than 3x the direct distance
    double efficiencyRatio = pathLength / directLength;
    bool isEfficient = efficiencyRatio <= 3.0;
    
    if (!isEfficient) {
        std::cout << "[PATH_VALIDATOR] Path efficiency warning: ratio " << efficiencyRatio << std::endl;
    }
    
    return isEfficient;
}

void PathValidator::setStrictValidation(bool strict) {
    strictValidation = strict;
    std::cout << "[PATH_VALIDATOR] Strict validation " << (strict ? "enabled" : "disabled") << std::endl;
}

void PathValidator::setMaxSegmentLength(double maxLength) {
    maxAllowedSegmentLength = maxLength;
    std::cout << "[PATH_VALIDATOR] Max segment length set to " << maxLength << std::endl;
}

void PathValidator::updateEnvironment(const Graph* newEnvironment) {
    if (!newEnvironment) {
        throw std::invalid_argument("New environment cannot be null");
    }
    
    environment = newEnvironment;
    std::cout << "[PATH_VALIDATOR] Environment updated" << std::endl;
}

double PathValidator::calculatePathRisk(const std::vector<int>& path) const {
    double safetyScore;
    std::string safetyReport;
    checkPathSafety(environment, path, safetyScore, safetyReport);
    
    return 1.0 - safetyScore; // Convert safety score to risk level
}

std::vector<int> PathValidator::identifyProblematicSegments(const std::vector<int>& path) const {
    std::vector<int> problematicSegments;
    
    if (path.size() < 2) return problematicSegments;
    
    for (size_t i = 1; i < path.size(); ++i) {
        bool hasIssues = false;
        
        // Check if nodes exist
        if (!environment->hasNode(path[i-1]) || !environment->hasNode(path[i])) {
            hasIssues = true;
        }
        // Check connectivity
        else if (!environment->hasEdge(path[i-1], path[i])) {
            hasIssues = true;
        }
        // Check segment length
        else {
            const Node& from = environment->getNode(path[i-1]);
            const Node& to = environment->getNode(path[i]);
            double segmentLength = from.euclideanDistance(to);
            
            if (segmentLength > maxAllowedSegmentLength) {
                hasIssues = true;
            }
        }
        
        if (hasIssues) {
            problematicSegments.push_back(static_cast<int>(i-1));
        }
    }
    
    std::cout << "[PATH_VALIDATOR] Identified " << problematicSegments.size() 
              << " problematic segments" << std::endl;
    
    return problematicSegments;
}

// Private helper methods implementation

bool PathValidator::validatePathStructure(const std::vector<int>& path) const {
    // Check for duplicate consecutive nodes
    for (size_t i = 1; i < path.size(); ++i) {
        if (path[i] == path[i-1]) {
            std::cout << "[PATH_VALIDATOR] Duplicate consecutive node found: " << path[i] << std::endl;
            return false;
        }
    }
    
    // Check path length constraints
    if (path.size() > 1000) {
        std::cout << "[PATH_VALIDATOR] Path too long: " << path.size() << " nodes" << std::endl;
        return false;
    }
    
    return true;
}

bool PathValidator::validatePathConnectivity(const std::vector<int>& path) const {
    return checkPathExists(path) && checkPathContinuity(path);
}

bool PathValidator::validatePathSafety(const std::vector<int>& path) const {
    double safetyScore;
    std::string safetyReport;
    bool isSafe = checkPathSafety(environment, path, safetyScore, safetyReport);
    
    return isSafe && safetyScore >= 0.6; // Minimum safety threshold
}

std::string PathValidator::generateValidationReport(const std::vector<int>& path) const {
    std::ostringstream report;
    
    report << "Path Validation Report\n";
    report << "======================\n";
    report << "Path length: " << path.size() << " nodes\n";
    
    if (!path.empty()) {
        report << "Start node: " << path.front() << "\n";
        report << "End node: " << path.back() << "\n";
        report << "Path length (distance): " << calculatePathLength(path) << "\n";
        report << "Direct distance: " << calculateDirectDistance(path.front(), path.back()) << "\n";
        
        // Efficiency ratio
        if (path.size() >= 2) {
            double efficiency = calculatePathLength(path) / 
                              calculateDirectDistance(path.front(), path.back());
            report << "Efficiency ratio: " << efficiency << "\n";
        }
    }
    
    // Validation results
    report << "\nValidation Results:\n";
    report << "Structure: " << (validatePathStructure(path) ? "PASS" : "FAIL") << "\n";
    report << "Connectivity: " << (validatePathConnectivity(path) ? "PASS" : "FAIL") << "\n";
    report << "Safety: " << (validatePathSafety(path) ? "PASS" : "FAIL") << "\n";
    
    // Problematic segments
    std::vector<int> problematic = identifyProblematicSegments(path);
    if (!problematic.empty()) {
        report << "\nProblematic segments: ";
        for (size_t i = 0; i < problematic.size(); ++i) {
            if (i > 0) report << ", ";
            report << problematic[i];
        }
        report << "\n";
    }
    
    return report.str();
}

double PathValidator::calculatePathLength(const std::vector<int>& path) const {
    if (path.size() < 2) return 0.0;
    
    double totalLength = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        if (environment->hasNode(path[i-1]) && environment->hasNode(path[i])) {
            const Node& from = environment->getNode(path[i-1]);
            const Node& to = environment->getNode(path[i]);
            totalLength += from.euclideanDistance(to);
        }
    }
    
    return totalLength;
}

double PathValidator::calculateDirectDistance(int startId, int endId) const {
    if (!environment->hasNode(startId) || !environment->hasNode(endId)) {
        return std::numeric_limits<double>::infinity();
    }
    
    const Node& start = environment->getNode(startId);
    const Node& end = environment->getNode(endId);
    return start.euclideanDistance(end);
}

// Global validation functions

bool validatePathComplete(const Graph* graph, const std::vector<int>& path, 
                         std::string& validationReport) {
    PathValidator validator(graph, true);
    return validator.validatePathWithDetails(path, validationReport);
}

bool isPathValid(const Graph* graph, const std::vector<int>& path) {
    PathValidator validator(graph);
    return validator.validatePath(path);
}

double assessPathRisk(const Graph* graph, const std::vector<int>& path) {
    PathValidator validator(graph);
    return validator.calculatePathRisk(path);
}

std::vector<int> findPathProblems(const Graph* graph, const std::vector<int>& path) {
    PathValidator validator(graph);
    return validator.identifyProblematicSegments(path);
}
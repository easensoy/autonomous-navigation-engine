#pragma once
#include "core/Graph.hpp"
#include <vector>
#include <string>

class PathValidator {
private:
    const Graph* environment;
    bool strictValidation;
    double maxAllowedSegmentLength;
    
    bool validatePathStructure(const std::vector<int>& path) const;
    bool validatePathConnectivity(const std::vector<int>& path) const;
    bool validatePathSafety(const std::vector<int>& path) const;
    std::string generateValidationReport(const std::vector<int>& path) const;

public:
    explicit PathValidator(const Graph* graph);
    PathValidator(const Graph* graph, bool strict);
    
    bool validatePath(const std::vector<int>& path) const;
    bool validatePathWithDetails(const std::vector<int>& path, std::string& errorMessage) const;
    bool isPathSafe(const std::vector<int>& path) const;
    
    bool checkPathExists(const std::vector<int>& path) const;
    bool checkPathContinuity(const std::vector<int>& path) const;
    bool checkPathEfficiency(const std::vector<int>& path) const;
    
    void setStrictValidation(bool strict);
    void setMaxSegmentLength(double maxLength);
    void updateEnvironment(const Graph* newEnvironment);
    
    double calculatePathRisk(const std::vector<int>& path) const;
    std::vector<int> identifyProblematicSegments(const std::vector<int>& path) const;
};
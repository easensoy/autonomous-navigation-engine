#pragma once
#include "core/Graph.hpp"
#include <unordered_set>
#include <vector>

enum class SmoothingMethod {
    BEZIER_CURVES,
    SPLINE_INTERPOLATION,
    MOVING_AVERAGE,
    GAUSSIAN_FILTER,
    CUBIC_INTERPOLATION
};

class PathSmoother {
private:
    const Graph* environment;
    SmoothingMethod currentMethod;
    double smoothingFactor;
    std::unordered_set<int> criticalWaypoints;  // never removed by smoothing
    

public:
    explicit PathSmoother(const Graph* graph);
    
    std::vector<int> smoothPath(const std::vector<int>& roughPath);
    std::vector<int> smoothWithMethod(const std::vector<int>& roughPath, SmoothingMethod method);
    std::vector<int> adaptiveSmoothing(const std::vector<int>& roughPath);
    
    void setSmoothingMethod(SmoothingMethod method);
    void setSmoothingFactor(double factor);
    
    double calculateSmoothness(const std::vector<int>& path) const;
    std::vector<int> identifySharpCorners(const std::vector<int>& path) const;
    bool validateSmoothedPath(const std::vector<int>& originalPath, const std::vector<int>& smoothedPath) const;
    
    // Nodes that smoothing never removes from a path; replaces any earlier set (an empty list clears it).
    void preserveCriticalWaypoints(const std::vector<int>& criticalPoints);
};
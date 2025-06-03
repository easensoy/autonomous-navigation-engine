#pragma once
#include "core/Graph.hpp"
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
    
    std::vector<int> bezierSmoothing(const std::vector<int>& roughPath);
    std::vector<int> splineSmoothing(const std::vector<int>& roughPath);
    std::vector<int> movingAverageSmoothing(const std::vector<int>& roughPath);

public:
    explicit PathSmoother(const Graph* graph);
    
    std::vector<int> smoothPath(const std::vector<int>& roughPath);
    std::vector<int> smoothWithMethod(const std::vector<int>& roughPath, SmoothingMethod method);
    std::vector<int> adaptiveSmoothing(const std::vector<int>& roughPath);
    
    void setSmoothingMethod(SmoothingMethod method);
    void setSmoothingFactor(double factor);
    void enableCornerSmoothing(bool enable);
    
    double calculateSmoothness(const std::vector<int>& path) const;
    std::vector<int> identifySharpCorners(const std::vector<int>& path) const;
    bool validateSmoothedPath(const std::vector<int>& originalPath, const std::vector<int>& smoothedPath) const;
    
    void setMinimumSegmentLength(double minLength);
    void preserveCriticalWaypoints(const std::vector<int>& criticalPoints);
};
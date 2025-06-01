#pragma once
#include <vector>
#include <cmath>

namespace MathUtils {
    
    constexpr double PI = 3.14159265358979323846;
    constexpr double EPSILON = 1e-9;
    
    double euclideanDistance(double x1, double y1, double x2, double y2);
    double manhattanDistance(double x1, double y1, double x2, double y2);
    double chebyshevDistance(double x1, double y1, double x2, double y2);
    
    double calculateAngle(double x1, double y1, double x2, double y2);
    double normalizeAngle(double angle);
    double angleDifference(double angle1, double angle2);
    
    bool isPointInPolygon(double x, double y, const std::vector<std::pair<double, double>>& polygon);
    bool doLinesIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4);
    
    double interpolateLinear(double x, double x1, double y1, double x2, double y2);
    std::vector<std::pair<double, double>> interpolatePath(const std::vector<std::pair<double, double>>& points, int numInterpolatedPoints);
    
    double calculateCurvature(double x1, double y1, double x2, double y2, double x3, double y3);
    std::vector<double> smoothData(const std::vector<double>& data, int windowSize);
    
    double clamp(double value, double minVal, double maxVal);
    bool approximately(double a, double b, double tolerance = EPSILON);
    double roundToPrecision(double value, int decimals);
    
    std::pair<double, double> rotatePoint(double x, double y, double angle, double centerX = 0.0, double centerY = 0.0);
    std::vector<std::pair<double, double>> generateCirclePoints(double centerX, double centerY, double radius, int numPoints);
    
}
#include "utilities/MathUtils.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace MathUtils {

double euclideanDistance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double manhattanDistance(double x1, double y1, double x2, double y2) {
    return std::abs(x2 - x1) + std::abs(y2 - y1);
}

double chebyshevDistance(double x1, double y1, double x2, double y2) {
    return std::max(std::abs(x2 - x1), std::abs(y2 - y1));
}

double calculateCurvature(double x1, double y1, double x2, double y2, double x3, double y3) {
    // Calculate curvature using the formula: k = |det(v1, v2)| / |v1|^3
    // where v1 is the first derivative and v2 is the second derivative
    
    double v1x = x2 - x1;
    double v1y = y2 - y1;
    double v2x = x3 - x2;
    double v2y = y3 - y2;
    
    double dx = v2x - v1x;
    double dy = v2y - v1y;
    
    double crossProduct = v1x * dy - v1y * dx;
    double magnitude = std::pow(v1x * v1x + v1y * v1y, 1.5);
    
    if (magnitude < EPSILON) {
        return 0.0;
    }
    
    return std::abs(crossProduct) / magnitude;
}

double clamp(double value, double minVal, double maxVal) {
    return std::max(minVal, std::min(value, maxVal));
}

bool approximately(double a, double b, double tolerance) {
    return std::abs(a - b) < tolerance;
}

double roundToPrecision(double value, int decimals) {
    double factor = std::pow(10.0, decimals);
    return std::round(value * factor) / factor;
}

std::pair<double, double> rotatePoint(double x, double y, double angle, double centerX, double centerY) {
    double cosAngle = std::cos(angle);
    double sinAngle = std::sin(angle);
    
    // Translate to origin
    double translatedX = x - centerX;
    double translatedY = y - centerY;
    
    // Rotate
    double rotatedX = translatedX * cosAngle - translatedY * sinAngle;
    double rotatedY = translatedX * sinAngle + translatedY * cosAngle;
    
    // Translate back
    return {rotatedX + centerX, rotatedY + centerY};
}

std::vector<std::pair<double, double>> generateCirclePoints(double centerX, double centerY, double radius, int numPoints) {
    std::vector<std::pair<double, double>> points;
    points.reserve(numPoints);
    
    for (int i = 0; i < numPoints; ++i) {
        double angle = 2.0 * PI * i / numPoints;
        double x = centerX + radius * std::cos(angle);
        double y = centerY + radius * std::sin(angle);
        points.emplace_back(x, y);
    }
    
    return points;
}

bool isPointInPolygon(double x, double y, const std::vector<std::pair<double, double>>& polygon) {
    if (polygon.size() < 3) {
        return false;
    }
    
    bool inside = false;
    size_t j = polygon.size() - 1;
    
    for (size_t i = 0; i < polygon.size(); ++i) {
        double xi = polygon[i].first;
        double yi = polygon[i].second;
        double xj = polygon[j].first;
        double yj = polygon[j].second;
        
        if (((yi > y) != (yj > y)) && 
            (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
        j = i;
    }
    
    return inside;
}

bool doLinesIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
    double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    
    if (std::abs(denom) < EPSILON) {
        return false; // Lines are parallel
    }
    
    double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
    
    return (t >= 0 && t <= 1 && u >= 0 && u <= 1);
}

double dotProduct(double x1, double y1, double x2, double y2) {
    return x1 * x2 + y1 * y2;
}

double crossProduct2D(double x1, double y1, double x2, double y2) {
    return x1 * y2 - y1 * x2;
}

double vectorMagnitude(double x, double y) {
    return std::sqrt(x * x + y * y);
}

std::pair<double, double> normalizeVector(double x, double y) {
    double magnitude = vectorMagnitude(x, y);
    if (magnitude < EPSILON) {
        return {0.0, 0.0};
    }
    return {x / magnitude, y / magnitude};
}

std::pair<double, double> vectorAdd(double x1, double y1, double x2, double y2) {
    return {x1 + x2, y1 + y2};
}

std::pair<double, double> vectorSubtract(double x1, double y1, double x2, double y2) {
    return {x1 - x2, y1 - y2};
}

std::pair<double, double> vectorScale(double x, double y, double scale) {
    return {x * scale, y * scale};
}

double pointToLineDistance(double px, double py, double x1, double y1, double x2, double y2) {
    double A = px - x1;
    double B = py - y1;
    double C = x2 - x1;
    double D = y2 - y1;
    
    double dot = A * C + B * D;
    double lenSq = C * C + D * D;
    
    if (lenSq < EPSILON) {
        return euclideanDistance(px, py, x1, y1);
    }
    
    double param = dot / lenSq;
    
    double xx, yy;
    if (param < 0) {
        xx = x1;
        yy = y1;
    } else if (param > 1) {
        xx = x2;
        yy = y2;
    } else {
        xx = x1 + param * C;
        yy = y1 + param * D;
    }
    
    return euclideanDistance(px, py, xx, yy);
}

std::pair<double, double> closestPointOnLine(double px, double py, double x1, double y1, double x2, double y2) {
    double A = px - x1;
    double B = py - y1;
    double C = x2 - x1;
    double D = y2 - y1;
    
    double dot = A * C + B * D;
    double lenSq = C * C + D * D;
    
    if (lenSq < EPSILON) {
        return {x1, y1};
    }
    
    double param = clamp(dot / lenSq, 0.0, 1.0);
    return {x1 + param * C, y1 + param * D};
}

bool isPointOnLineSegment(double px, double py, double x1, double y1, double x2, double y2, double tolerance) {
    double distance = pointToLineDistance(px, py, x1, y1, x2, y2);
    return distance <= tolerance;
}

double calculateArea(const std::vector<std::pair<double, double>>& polygon) {
    if (polygon.size() < 3) {
        return 0.0;
    }
    
    double area = 0.0;
    size_t n = polygon.size();
    
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        area += polygon[i].first * polygon[j].second;
        area -= polygon[j].first * polygon[i].second;
    }
    
    return std::abs(area) / 2.0;
}

std::pair<double, double> calculateCentroid(const std::vector<std::pair<double, double>>& points) {
    if (points.empty()) {
        return {0.0, 0.0};
    }
    
    double sumX = 0.0;
    double sumY = 0.0;
    
    for (const auto& point : points) {
        sumX += point.first;
        sumY += point.second;
    }
    
    return {sumX / points.size(), sumY / points.size()};
}

std::pair<std::pair<double, double>, std::pair<double, double>> getBoundingBox(const std::vector<std::pair<double, double>>& points) {
    if (points.empty()) {
        return {{0.0, 0.0}, {0.0, 0.0}};
    }
    
    double minX = points[0].first;
    double maxX = points[0].first;
    double minY = points[0].second;
    double maxY = points[0].second;
    
    for (const auto& point : points) {
        minX = std::min(minX, point.first);
        maxX = std::max(maxX, point.first);
        minY = std::min(minY, point.second);
        maxY = std::max(maxY, point.second);
    }
    
    return {{minX, minY}, {maxX, maxY}};
}

bool isConvexPolygon(const std::vector<std::pair<double, double>>& polygon) {
    if (polygon.size() < 3) {
        return false;
    }
    
    bool hasPositive = false;
    bool hasNegative = false;
    
    for (size_t i = 0; i < polygon.size(); ++i) {
        size_t j = (i + 1) % polygon.size();
        size_t k = (i + 2) % polygon.size();
        
        double v1x = polygon[j].first - polygon[i].first;
        double v1y = polygon[j].second - polygon[i].second;
        double v2x = polygon[k].first - polygon[j].first;
        double v2y = polygon[k].second - polygon[j].second;
        
        double cross = crossProduct2D(v1x, v1y, v2x, v2y);
        
        if (cross > EPSILON) hasPositive = true;
        if (cross < -EPSILON) hasNegative = true;
        
        if (hasPositive && hasNegative) {
            return false;
        }
    }
    
    return true;
}

}  // namespace MathUtils
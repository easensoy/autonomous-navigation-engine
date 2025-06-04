#include "utilities/MathUtils.hpp"
#include <cmath>
#include <algorithm>

namespace MathUtils {

double calculateAngle(double x1, double y1, double x2, double y2) {
    double deltaX = x2 - x1;
    double deltaY = y2 - y1;
    
    if (std::abs(deltaX) < EPSILON && std::abs(deltaY) < EPSILON) {
        return 0.0; // Points are the same
    }
    
    return std::atan2(deltaY, deltaX);
}

double normalizeAngle(double angle) {
    // Normalize angle to range [-PI, PI]
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}

double angleDifference(double angle1, double angle2) {
    double diff = angle2 - angle1;
    return normalizeAngle(diff);
}

double calculateBearing(double x1, double y1, double x2, double y2) {
    double angle = calculateAngle(x1, y1, x2, y2);
    // Convert from mathematical angle to bearing (0° = North)
    double bearing = (PI / 2.0) - angle;
    return normalizeAngle(bearing);
}

double degreesToRadians(double degrees) {
    return degrees * PI / 180.0;
}

double radiansToDegrees(double radians) {
    return radians * 180.0 / PI;
}

double calculateAngleBetweenVectors(double x1, double y1, double x2, double y2) {
    double dot = x1 * x2 + y1 * y2;
    double mag1 = std::sqrt(x1 * x1 + y1 * y1);
    double mag2 = std::sqrt(x2 * x2 + y2 * y2);
    
    if (mag1 < EPSILON || mag2 < EPSILON) {
        return 0.0;
    }
    
    double cosAngle = dot / (mag1 * mag2);
    cosAngle = std::clamp(cosAngle, -1.0, 1.0); // Handle floating point errors
    
    return std::acos(cosAngle);
}

double calculateTurnAngle(double x1, double y1, double x2, double y2, double x3, double y3) {
    // Calculate vectors
    double v1x = x2 - x1;
    double v1y = y2 - y1;
    double v2x = x3 - x2;
    double v2y = y3 - y2;
    
    // Calculate angle between vectors
    double angle1 = std::atan2(v1y, v1x);
    double angle2 = std::atan2(v2y, v2x);
    
    return angleDifference(angle1, angle2);
}

bool isAngleAcute(double angle) {
    double absAngle = std::abs(normalizeAngle(angle));
    return absAngle < PI / 2.0;
}

bool isAngleObtuse(double angle) {
    double absAngle = std::abs(normalizeAngle(angle));
    return absAngle > PI / 2.0 && absAngle < PI;
}

double calculateSlope(double x1, double y1, double x2, double y2) {
    if (std::abs(x2 - x1) < EPSILON) {
        return std::numeric_limits<double>::infinity(); // Vertical line
    }
    return (y2 - y1) / (x2 - x1);
}

double calculateSlopeAngle(double x1, double y1, double x2, double y2) {
    double slope = calculateSlope(x1, y1, x2, y2);
    if (std::isinf(slope)) {
        return PI / 2.0; // 90 degrees for vertical line
    }
    return std::atan(slope);
}

std::pair<double, double> getUnitVector(double angle) {
    return {std::cos(angle), std::sin(angle)};
}

double calculateCompassBearing(double x1, double y1, double x2, double y2) {
    double angle = calculateAngle(x1, y1, x2, y2);
    // Convert to compass bearing (0° = North, clockwise)
    double bearing = 90.0 - radiansToDegrees(angle);
    
    // Normalize to [0, 360)
    while (bearing < 0.0) bearing += 360.0;
    while (bearing >= 360.0) bearing -= 360.0;
    
    return bearing;
}

double interpolateAngle(double angle1, double angle2, double t) {
    // Handle wraparound for angle interpolation
    double diff = angleDifference(angle1, angle2);
    return normalizeAngle(angle1 + t * diff);
}

}  // namespace MathUtils
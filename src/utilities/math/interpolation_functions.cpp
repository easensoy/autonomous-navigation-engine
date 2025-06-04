#include "utilities/MathUtils.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace MathUtils {

double interpolateLinear(double x, double x1, double y1, double x2, double y2) {
    if (std::abs(x2 - x1) < EPSILON) {
        return y1; // Points have same x-coordinate
    }
    
    double t = (x - x1) / (x2 - x1);
    return y1 + t * (y2 - y1);
}

double interpolateBilinear(double x, double y, 
                          double x1, double y1, double v11,
                          double x2, double y1_val, double v21,
                          double x1_val, double y2, double v12,
                          double x2_val, double y2_val, double v22) {
    
    double tx = (x - x1) / (x2 - x1);
    double ty = (y - y1) / (y2 - y1);
    
    double top = v11 * (1 - tx) + v21 * tx;
    double bottom = v12 * (1 - tx) + v22 * tx;
    
    return top * (1 - ty) + bottom * ty;
}

double interpolateCubic(double t, double p0, double p1, double p2, double p3) {
    double t2 = t * t;
    double t3 = t2 * t;
    
    return 0.5 * ((2 * p1) +
                  (-p0 + p2) * t +
                  (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 +
                  (-p0 + 3 * p1 - 3 * p2 + p3) * t3);
}

std::vector<std::pair<double, double>> interpolatePath(const std::vector<std::pair<double, double>>& points, int numInterpolatedPoints) {
    std::vector<std::pair<double, double>> result;
    
    if (points.size() < 2) {
        return points;
    }
    
    if (numInterpolatedPoints <= 0) {
        return points;
    }
    
    result.reserve(numInterpolatedPoints);
    
    // Calculate total path length
    double totalLength = 0.0;
    std::vector<double> segmentLengths;
    
    for (size_t i = 1; i < points.size(); ++i) {
        double dx = points[i].first - points[i-1].first;
        double dy = points[i].second - points[i-1].second;
        double length = std::sqrt(dx * dx + dy * dy);
        segmentLengths.push_back(length);
        totalLength += length;
    }
    
    // Interpolate points based on arc length
    double stepLength = totalLength / (numInterpolatedPoints - 1);
    double currentLength = 0.0;
    size_t segmentIndex = 0;
    
    result.push_back(points[0]); // Add first point
    
    for (int i = 1; i < numInterpolatedPoints - 1; ++i) {
        double targetLength = i * stepLength;
        
        // Find which segment contains this target length
        while (segmentIndex < segmentLengths.size() && 
               currentLength + segmentLengths[segmentIndex] < targetLength) {
            currentLength += segmentLengths[segmentIndex];
            segmentIndex++;
        }
        
        if (segmentIndex >= segmentLengths.size()) {
            break;
        }
        
        // Interpolate within the segment
        double segmentProgress = (targetLength - currentLength) / segmentLengths[segmentIndex];
        
        double x = points[segmentIndex].first + 
                   segmentProgress * (points[segmentIndex + 1].first - points[segmentIndex].first);
        double y = points[segmentIndex].second + 
                   segmentProgress * (points[segmentIndex + 1].second - points[segmentIndex].second);
        
        result.emplace_back(x, y);
    }
    
    result.push_back(points.back()); // Add last point
    return result;
}

std::vector<std::pair<double, double>> interpolateBezier(const std::vector<std::pair<double, double>>& controlPoints, int numPoints) {
    std::vector<std::pair<double, double>> result;
    
    if (controlPoints.size() < 2) {
        return controlPoints;
    }
    
    result.reserve(numPoints);
    
    for (int i = 0; i < numPoints; ++i) {
        double t = static_cast<double>(i) / (numPoints - 1);
        
        // De Casteljau's algorithm for Bezier curves
        std::vector<std::pair<double, double>> tempPoints = controlPoints;
        
        for (size_t level = controlPoints.size() - 1; level > 0; --level) {
            for (size_t j = 0; j < level; ++j) {
                tempPoints[j].first = (1 - t) * tempPoints[j].first + t * tempPoints[j + 1].first;
                tempPoints[j].second = (1 - t) * tempPoints[j].second + t * tempPoints[j + 1].second;
            }
        }
        
        result.push_back(tempPoints[0]);
    }
    
    return result;
}

std::vector<double> smoothData(const std::vector<double>& data, int windowSize) {
    std::vector<double> smoothed;
    
    if (data.empty() || windowSize <= 0) {
        return data;
    }
    
    smoothed.reserve(data.size());
    
    int halfWindow = windowSize / 2;
    
    for (size_t i = 0; i < data.size(); ++i) {
        int start = std::max(0, static_cast<int>(i) - halfWindow);
        int end = std::min(static_cast<int>(data.size()) - 1, static_cast<int>(i) + halfWindow);
        
        double sum = 0.0;
        int count = 0;
        
        for (int j = start; j <= end; ++j) {
            sum += data[j];
            count++;
        }
        
        smoothed.push_back(sum / count);
    }
    
    return smoothed;
}

std::vector<double> smoothDataGaussian(const std::vector<double>& data, double sigma) {
    std::vector<double> smoothed;
    
    if (data.empty() || sigma <= 0.0) {
        return data;
    }
    
    smoothed.reserve(data.size());
    
    int kernelSize = static_cast<int>(6 * sigma); // 3 sigma on each side
    if (kernelSize % 2 == 0) kernelSize++; // Make it odd
    
    int halfKernel = kernelSize / 2;
    
    // Generate Gaussian kernel
    std::vector<double> kernel(kernelSize);
    double sum = 0.0;
    
    for (int i = 0; i < kernelSize; ++i) {
        int x = i - halfKernel;
        kernel[i] = std::exp(-(x * x) / (2.0 * sigma * sigma));
        sum += kernel[i];
    }
    
    // Normalize kernel
    for (double& k : kernel) {
        k /= sum;
    }
    
    // Apply convolution
    for (size_t i = 0; i < data.size(); ++i) {
        double value = 0.0;
        double weightSum = 0.0;
        
        for (int j = 0; j < kernelSize; ++j) {
            int dataIndex = static_cast<int>(i) + j - halfKernel;
            
            if (dataIndex >= 0 && dataIndex < static_cast<int>(data.size())) {
                value += data[dataIndex] * kernel[j];
                weightSum += kernel[j];
            }
        }
        
        smoothed.push_back(value / weightSum);
    }
    
    return smoothed;
}

double interpolateSpline(double x, const std::vector<double>& xPoints, const std::vector<double>& yPoints) {
    if (xPoints.size() != yPoints.size() || xPoints.size() < 2) {
        return 0.0;
    }
    
    // Find the interval containing x
    size_t i = 0;
    while (i < xPoints.size() - 1 && x > xPoints[i + 1]) {
        i++;
    }
    
    if (i >= xPoints.size() - 1) {
        return yPoints.back();
    }
    
    // Linear interpolation (can be extended to cubic spline)
    double t = (x - xPoints[i]) / (xPoints[i + 1] - xPoints[i]);
    return yPoints[i] + t * (yPoints[i + 1] - yPoints[i]);
}

std::pair<double, double> interpolatePointOnPath(const std::vector<std::pair<double, double>>& path, double t) {
    if (path.empty()) {
        return {0.0, 0.0};
    }
    
    if (path.size() == 1) {
        return path[0];
    }
    
    t = clamp(t, 0.0, 1.0);
    
    // Calculate cumulative distances
    std::vector<double> distances;
    distances.push_back(0.0);
    
    double totalLength = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].first - path[i-1].first;
        double dy = path[i].second - path[i-1].second;
        double length = std::sqrt(dx * dx + dy * dy);
        totalLength += length;
        distances.push_back(totalLength);
    }
    
    if (totalLength < EPSILON) {
        return path[0];
    }
    
    double targetDistance = t * totalLength;
    
    // Find segment
    size_t i = 0;
    while (i < distances.size() - 1 && distances[i + 1] < targetDistance) {
        i++;
    }
    
    if (i >= path.size() - 1) {
        return path.back();
    }
    
    // Interpolate within segment
    double segmentLength = distances[i + 1] - distances[i];
    if (segmentLength < EPSILON) {
        return path[i];
    }
    
    double segmentT = (targetDistance - distances[i]) / segmentLength;
    
    return {
        path[i].first + segmentT * (path[i + 1].first - path[i].first),
        path[i].second + segmentT * (path[i + 1].second - path[i].second)
    };
}

}  // namespace MathUtils
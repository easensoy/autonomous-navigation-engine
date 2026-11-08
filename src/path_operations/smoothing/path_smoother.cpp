#include "path_operations/PathSmoother.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

// PathSmoother works on node routes, so the result must still follow graph edges. A geometric
// curve (Bezier, spline) would leave the graph, so every method here cuts corners instead: a
// middle node is dropped when a direct edge joins its neighbours and is no longer than the corner.
// The smoothing factor sets how sharp a turn must be before it is cut (0 = every turn, 1 = none).

namespace {
const double kPi = 3.14159265358979323846;

double turnAngle(const Graph& graph, int a, int b, int c) {
    const Node& na = graph.getNode(a);
    const Node& nb = graph.getNode(b);
    const Node& nc = graph.getNode(c);
    const double ux = nb.getX() - na.getX(), uy = nb.getY() - na.getY();
    const double vx = nc.getX() - nb.getX(), vy = nc.getY() - nb.getY();
    const double lu = std::hypot(ux, uy), lv = std::hypot(vx, vy);
    if (lu == 0.0 || lv == 0.0) {
        return 0.0;
    }
    const double cosine = std::clamp((ux * vx + uy * vy) / (lu * lv), -1.0, 1.0);
    return std::acos(cosine);  // 0 = straight on, pi = full reversal
}
}  // namespace

PathSmoother::PathSmoother(const Graph* graph)
    : environment(graph), currentMethod(SmoothingMethod::BEZIER_CURVES), smoothingFactor(0.0) {
    if (!environment) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
}

std::vector<int> PathSmoother::smoothPath(const std::vector<int>& roughPath) {
    return smoothWithMethod(roughPath, currentMethod);
}

std::vector<int> PathSmoother::smoothWithMethod(const std::vector<int>& roughPath, SmoothingMethod /*method*/) {
    if (roughPath.size() < 3) {
        return roughPath;
    }
    const double minAngleToCut = smoothingFactor * kPi;
    std::vector<int> path = roughPath;
    bool changed = true;
    while (changed) {
        changed = false;
        for (size_t i = 1; i + 1 < path.size(); ++i) {
            const int a = path[i - 1], b = path[i], c = path[i + 1];
            if (criticalWaypoints.count(b) || !environment->hasEdge(a, c) ||
                turnAngle(*environment, a, b, c) <= minAngleToCut) {
                continue;
            }
            const double corner = environment->getEdgeWeight(a, b) + environment->getEdgeWeight(b, c);
            if (environment->getEdgeWeight(a, c) <= corner) {
                path.erase(path.begin() + static_cast<long>(i));
                changed = true;
                break;
            }
        }
    }
    return path;
}

std::vector<int> PathSmoother::adaptiveSmoothing(const std::vector<int>& roughPath) {
    // Cut only the turns sharper than the current smoothing factor; never more than a right angle.
    const double saved = smoothingFactor;
    smoothingFactor = std::max(smoothingFactor, 0.5);
    std::vector<int> result = smoothWithMethod(roughPath, currentMethod);
    smoothingFactor = saved;
    return result;
}

void PathSmoother::setSmoothingMethod(SmoothingMethod method) {
    currentMethod = method;
}

void PathSmoother::setSmoothingFactor(double factor) {
    smoothingFactor = std::clamp(factor, 0.0, 1.0);
}

double PathSmoother::calculateSmoothness(const std::vector<int>& path) const {
    if (path.size() < 3) {
        return 1.0;
    }
    double totalTurn = 0.0;
    for (size_t i = 1; i + 1 < path.size(); ++i) {
        totalTurn += turnAngle(*environment, path[i - 1], path[i], path[i + 1]);
    }
    const double averageTurn = totalTurn / static_cast<double>(path.size() - 2);
    return 1.0 - averageTurn / kPi;  // 1 = perfectly straight
}

std::vector<int> PathSmoother::identifySharpCorners(const std::vector<int>& path) const {
    std::vector<int> corners;
    for (size_t i = 1; i + 1 < path.size(); ++i) {
        if (turnAngle(*environment, path[i - 1], path[i], path[i + 1]) > kPi / 3.0) {
            corners.push_back(path[i]);
        }
    }
    return corners;
}

bool PathSmoother::validateSmoothedPath(const std::vector<int>& originalPath,
                                        const std::vector<int>& smoothedPath) const {
    if (originalPath.empty() || smoothedPath.empty() || originalPath.front() != smoothedPath.front() ||
        originalPath.back() != smoothedPath.back()) {
        return false;
    }
    for (size_t i = 1; i < smoothedPath.size(); ++i) {
        if (!environment->hasEdge(smoothedPath[i - 1], smoothedPath[i])) {
            return false;
        }
    }
    return true;
}

void PathSmoother::preserveCriticalWaypoints(const std::vector<int>& criticalPoints) {
    criticalWaypoints = std::unordered_set<int>(criticalPoints.begin(), criticalPoints.end());
}

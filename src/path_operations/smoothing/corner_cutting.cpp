#include "path_operations/PathSmoother.hpp"
#include "utilities/MathUtils.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

class CornerCuttingSmoother {
private:
    const Graph* graph;
    
    struct Corner {
        int nodeIndex;
        int nodeId;
        double angle;
        double sharpness;
        bool canCut;
        std::vector<int> alternativeNodes;
        double cuttingRadius;
        
        Corner(int index, int id) : nodeIndex(index), nodeId(id), angle(0.0), 
                                   sharpness(0.0), canCut(false), cuttingRadius(0.0) {}
    };
    
    struct CuttingResult {
        std::vector<int> smoothedPath;
        int cornersCut;
        double totalAngleReduction;
        double pathLengthChange;
        
        CuttingResult() : cornersCut(0), totalAngleReduction(0.0), pathLengthChange(0.0) {}
    };
    
    double maxCuttingRadius;
    double minCornerAngle;
    double cuttingAggressiveness;
    bool preserveEndpoints;
    std::vector<int> protectedWaypoints;
    
    std::vector<Corner> identifyCorners(const std::vector<int>& path) const {
        std::vector<Corner> corners;
        
        for (size_t i = 1; i < path.size() - 1; ++i) {
            Corner corner(static_cast<int>(i), path[i]);
            
            // Calculate angle at this corner
            corner.angle = calculateCornerAngle(path, i);
            corner.sharpness = M_PI - corner.angle; // Deviation from straight line
            
            // Determine if corner can be cut
            corner.canCut = (corner.sharpness > minCornerAngle) && 
                           !isProtectedWaypoint(corner.nodeId) &&
                           !(preserveEndpoints && (i == 1 || i == path.size() - 2));
            
            if (corner.canCut) {
                corner.cuttingRadius = calculateOptimalCuttingRadius(path, i);
                corner.alternativeNodes = findAlternativeNodes(path, i, corner.cuttingRadius);
            }
            
            corners.push_back(corner);
        }
        
        return corners;
    }
    
    double calculateCornerAngle(const std::vector<int>& path, size_t index) const {
        if (index == 0 || index >= path.size() - 1) return M_PI;
        
        const Node& prev = graph->getNode(path[index - 1]);
        const Node& curr = graph->getNode(path[index]);
        const Node& next = graph->getNode(path[index + 1]);
        
        // Vector from current to previous
        double v1x = prev.getX() - curr.getX();
        double v1y = prev.getY() - curr.getY();
        
        // Vector from current to next
        double v2x = next.getX() - curr.getX();
        double v2y = next.getY() - curr.getY();
        
        // Calculate angle between vectors
        double dot = v1x * v2x + v1y * v2y;
        double mag1 = std::sqrt(v1x * v1x + v1y * v1y);
        double mag2 = std::sqrt(v2x * v2x + v2y * v2y);
        
        if (mag1 == 0 || mag2 == 0) return M_PI;
        
        double cosAngle = dot / (mag1 * mag2);
        cosAngle = MathUtils::clamp(cosAngle, -1.0, 1.0);
        
        return std::acos(cosAngle);
    }
    
    double calculateOptimalCuttingRadius(const std::vector<int>& path, size_t cornerIndex) const {
        const Node& prev = graph->getNode(path[cornerIndex - 1]);
        const Node& curr = graph->getNode(path[cornerIndex]);
        const Node& next = graph->getNode(path[cornerIndex + 1]);
        
        double dist1 = prev.euclideanDistance(curr);
        double dist2 = curr.euclideanDistance(next);
        
        // Cutting radius is limited by shorter segment and max radius
        double maxRadius = std::min({dist1, dist2, maxCuttingRadius}) * 0.5;
        
        // Adjust based on corner sharpness
        double angle = calculateCornerAngle(path, cornerIndex);
        double sharpnessFactor = (M_PI - angle) / M_PI;
        
        return maxRadius * sharpnessFactor * cuttingAggressiveness;
    }
    
    std::vector<int> findAlternativeNodes(const std::vector<int>& path, size_t cornerIndex, 
                                        double radius) const {
        std::vector<int> alternatives;
        const Node& cornerNode = graph->getNode(path[cornerIndex]);
        
        // Find nodes within cutting radius that could provide smoother path
        for (int nodeId : graph->getAllNodeIds()) {
            if (nodeId == path[cornerIndex]) continue;
            
            const Node& candidateNode = graph->getNode(nodeId);
            double distance = cornerNode.euclideanDistance(candidateNode);
            
            if (distance <= radius) {
                // Check if this node would create a smoother path
                if (wouldImprovePathSmoothness(path, cornerIndex, nodeId)) {
                    alternatives.push_back(nodeId);
                }
            }
        }
        
        return alternatives;
    }
    
    bool wouldImprovePathSmoothness(const std::vector<int>& path, size_t cornerIndex, 
                                  int alternativeNode) const {
        if (cornerIndex == 0 || cornerIndex >= path.size() - 1) return false;
        
        // Calculate current angle
        double currentAngle = calculateCornerAngle(path, cornerIndex);
        
        // Calculate angle with alternative node
        const Node& prev = graph->getNode(path[cornerIndex - 1]);
        const Node& alt = graph->getNode(alternativeNode);
        const Node& next = graph->getNode(path[cornerIndex + 1]);
        
        double v1x = prev.getX() - alt.getX();
        double v1y = prev.getY() - alt.getY();
        double v2x = next.getX() - alt.getX();
        double v2y = next.getY() - alt.getY();
        
        double dot = v1x * v2x + v1y * v2y;
        double mag1 = std::sqrt(v1x * v1x + v1y * v1y);
        double mag2 = std::sqrt(v2x * v2x + v2y * v2y);
        
        if (mag1 == 0 || mag2 == 0) return false;
        
        double cosAngle = MathUtils::clamp(dot / (mag1 * mag2), -1.0, 1.0);
        double alternativeAngle = std::acos(cosAngle);
        
        // Alternative is better if it creates a straighter path
        return alternativeAngle > currentAngle;
    }
    
    CuttingResult performCornerCutting(const std::vector<int>& originalPath) {
        CuttingResult result;
        result.smoothedPath = originalPath;
        
        std::vector<Corner> corners = identifyCorners(originalPath);
        
        // Sort corners by sharpness (cut sharpest first)
        std::sort(corners.begin(), corners.end(),
                 [](const Corner& a, const Corner& b) {
                     return a.sharpness > b.sharpness;
                 });
        
        std::cout << "[CORNER_CUTTING] Found " << corners.size() << " corners, " 
                  << std::count_if(corners.begin(), corners.end(), 
                                  [](const Corner& c) { return c.canCut; })
                  << " can be cut" << std::endl;
        
        for (const Corner& corner : corners) {
            if (!corner.canCut || corner.alternativeNodes.empty()) continue;
            
            // Find best alternative node for this corner
            int bestAlternative = selectBestAlternative(corner, result.smoothedPath);
            
            if (bestAlternative != -1) {
                // Replace corner node with alternative
                auto it = std::find(result.smoothedPath.begin(), result.smoothedPath.end(), corner.nodeId);
                if (it != result.smoothedPath.end()) {
                    // Check connectivity before replacing
                    if (validateCornerCut(result.smoothedPath, it - result.smoothedPath.begin(), bestAlternative)) {
                        *it = bestAlternative;
                        result.cornersCut++;
                        result.totalAngleReduction += corner.sharpness;
                        
                        std::cout << "[CORNER_CUTTING] Cut corner at node " << corner.nodeId 
                                  << " with alternative " << bestAlternative << std::endl;
                    }
                }
            }
        }
        
        // Clean up any connectivity issues
        result.smoothedPath = ensurePathConnectivity(result.smoothedPath);
        
        // Calculate metrics
        result.pathLengthChange = calculatePathLength(result.smoothedPath) - 
                                calculatePathLength(originalPath);
        
        return result;
    }
    
    int selectBestAlternative(const Corner& corner, const std::vector<int>& currentPath) const {
        if (corner.alternativeNodes.empty()) return -1;
        
        int bestNode = -1;
        double bestScore = -1.0;
        
        for (int altNode : corner.alternativeNodes) {
            double score = evaluateAlternativeNode(corner, altNode, currentPath);
            if (score > bestScore) {
                bestScore = score;
                bestNode = altNode;
            }
        }
        
        return bestNode;
    }
    
    double evaluateAlternativeNode(const Corner& corner, int altNode, 
                                 const std::vector<int>& currentPath) const {
        double score = 0.0;
        
        // Find corner position in current path
        auto it = std::find(currentPath.begin(), currentPath.end(), corner.nodeId);
        if (it == currentPath.end() || it == currentPath.begin() || it == currentPath.end() - 1) {
            return -1.0;
        }
        
        size_t index = it - currentPath.begin();
        
        // Score based on angle improvement
        double currentAngle = calculateCornerAngle(currentPath, index);
        
        // Calculate angle with alternative (simulate replacement)
        std::vector<int> testPath = currentPath;
        testPath[index] = altNode;
        double altAngle = calculateCornerAngle(testPath, index);
        
        score += (altAngle - currentAngle) * 10.0; // Favor straighter paths
        
        // Score based on path length change
        double currentLength = calculateSegmentLength(currentPath[index-1], currentPath[index]) +
                              calculateSegmentLength(currentPath[index], currentPath[index+1]);
        double altLength = calculateSegmentLength(currentPath[index-1], altNode) +
                          calculateSegmentLength(altNode, currentPath[index+1]);
        
        score += (currentLength - altLength); // Favor shorter paths
        
        // Penalty for nodes that are too far from original
        const Node& original = graph->getNode(corner.nodeId);
        const Node& alternative = graph->getNode(altNode);
        double displacement = original.euclideanDistance(alternative);
        score -= displacement * 0.5;
        
        return score;
    }
    
    bool validateCornerCut(const std::vector<int>& path, size_t index, int newNode) const {
        if (index == 0 || index >= path.size() - 1) return false;
        
        // Check connectivity
        bool connectedToPrev = graph->hasEdge(path[index - 1], newNode) ||
                              findShortestPath(path[index - 1], newNode).size() <= 3;
        bool connectedToNext = graph->hasEdge(newNode, path[index + 1]) ||
                              findShortestPath(newNode, path[index + 1]).size() <= 3;
        
        return connectedToPrev && connectedToNext;
    }
    
    double calculatePathLength(const std::vector<int>& path) const {
        if (path.size() < 2) return 0.0;
        
        double length = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            length += calculateSegmentLength(path[i-1], path[i]);
        }
        return length;
    }
    
    double calculateSegmentLength(int nodeId1, int nodeId2) const {
        const Node& node1 = graph->getNode(nodeId1);
        const Node& node2 = graph->getNode(nodeId2);
        return node1.euclideanDistance(node2);
    }
    
    bool isProtectedWaypoint(int nodeId) const {
        return std::find(protectedWaypoints.begin(), protectedWaypoints.end(), nodeId) 
               != protectedWaypoints.end();
    }
    
public:
    CornerCuttingSmoother(const Graph* environment) 
        : graph(environment), maxCuttingRadius(5.0), minCornerAngle(M_PI/6),
          cuttingAggressiveness(0.7), preserveEndpoints(true) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[CORNER_CUTTING] Corner cutting smoother initialized" << std::endl;
    }
    
    std::vector<int> smoothPath(const std::vector<int>& originalPath) {
        if (originalPath.size() < 3) {
            std::cout << "[CORNER_CUTTING] Path too short for corner cutting" << std::endl;
            return originalPath;
        }
        
        std::cout << "[CORNER_CUTTING] Applying corner cutting to " 
                  << originalPath.size() << " nodes" << std::endl;
        
        CuttingResult result = performCornerCutting(originalPath);
        
        std::cout << "[CORNER_CUTTING] Corner cutting completed: cut " << result.cornersCut 
                  << " corners, path length change: " << result.pathLengthChange << std::endl;
        
        return result.smoothedPath;
    }
    
    void setMaxCuttingRadius(double radius) {
        maxCuttingRadius = radius;
        std::cout << "[CORNER_CUTTING] Max cutting radius set to " << radius << std::endl;
    }
    
    void setMinCornerAngle(double angle) {
        minCornerAngle = angle;
        std::cout << "[CORNER_CUTTING] Min corner angle set to " << angle << " radians" << std::endl;
    }
    
    void setCuttingAggressiveness(double aggressiveness) {
        cuttingAggressiveness = MathUtils::clamp(aggressiveness, 0.0, 1.0);
        std::cout << "[CORNER_CUTTING] Cutting aggressiveness set to " << aggressiveness << std::endl;
    }
    
    void setProtectedWaypoints(const std::vector<int>& waypoints) {
        protectedWaypoints = waypoints;
        std::cout << "[CORNER_CUTTING] Set " << waypoints.size() << " protected waypoints" << std::endl;
    }
    
private:
    std::vector<int> ensurePathConnectivity(const std::vector<int>& path) const {
        std::vector<int> connectedPath;
        if (path.empty()) return connectedPath;
        
        connectedPath.push_back(path[0]);
        
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasEdge(connectedPath.back(), path[i])) {
                std::vector<int> bridge = findShortestPath(connectedPath.back(), path[i]);
                if (!bridge.empty() && bridge.size() > 1) {
                    connectedPath.insert(connectedPath.end(), bridge.begin() + 1, bridge.end());
                } else {
                    connectedPath.push_back(path[i]);
                }
            } else {
                connectedPath.push_back(path[i]);
            }
        }
        
        return connectedPath;
    }
    
    std::vector<int> findShortestPath(int start, int end) const {
        std::queue<std::vector<int>> pathQueue;
        std::unordered_set<int> visited;
        
        pathQueue.push({start});
        visited.insert(start);
        
        while (!pathQueue.empty()) {
            std::vector<int> currentPath = pathQueue.front();
            pathQueue.pop();
            
            if (currentPath.back() == end) {
                return currentPath;
            }
            
            if (currentPath.size() > 5) continue;
            
            for (int neighbor : graph->getNeighbors(currentPath.back())) {
                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    std::vector<int> newPath = currentPath;
                    newPath.push_back(neighbor);
                    pathQueue.push(newPath);
                }
            }
        }
        
        return {};
    }
};
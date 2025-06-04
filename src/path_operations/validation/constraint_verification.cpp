#include "path_operations/PathValidator.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>

class ConstraintVerifier {
private:
    const Graph* graph;
    bool strictMode;
    double maxSegmentLength;
    double maxTotalLength;
    double maxDeviationAngle;
    size_t maxPathLength;
    
    struct ConstraintViolation {
        std::string type;
        std::string description;
        int segmentIndex;
        double severity;
        
        ConstraintViolation(const std::string& t, const std::string& desc, int index = -1, double sev = 1.0)
            : type(t), description(desc), segmentIndex(index), severity(sev) {}
    };
    
public:
    ConstraintVerifier(const Graph* environment, bool strict = true) 
        : graph(environment), strictMode(strict), maxSegmentLength(50.0), 
          maxTotalLength(1000.0), maxDeviationAngle(M_PI), maxPathLength(1000) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[CONSTRAINT_VERIFY] Constraint verifier initialized" << std::endl;
    }
    
    bool verifyPathConstraints(const std::vector<int>& path, std::vector<ConstraintViolation>& violations) const {
        violations.clear();
        
        if (path.empty()) {
            violations.emplace_back("structure", "Path is empty", -1, 1.0);
            return false;
        }
        
        std::cout << "[CONSTRAINT_VERIFY] Verifying constraints for path with " << path.size() << " nodes" << std::endl;
        
        bool isValid = true;
        
        // Check node existence constraint
        if (!verifyNodeExistence(path, violations)) {
            isValid = false;
        }
        
        // Check path length constraint
        if (!verifyPathLength(path, violations)) {
            isValid = false;
        }
        
        // Check segment length constraints
        if (!verifySegmentLengths(path, violations)) {
            isValid = false;
        }
        
        // Check total path length constraint
        if (!verifyTotalLength(path, violations)) {
            isValid = false;
        }
        
        // Check angular constraints
        if (!verifyAngularConstraints(path, violations)) {
            isValid = false;
        }
        
        // Check connectivity constraints
        if (!verifyConnectivityConstraints(path, violations)) {
            isValid = false;
        }
        
        // Check geometric constraints
        if (!verifyGeometricConstraints(path, violations)) {
            isValid = false;
        }
        
        if (isValid) {
            std::cout << "[CONSTRAINT_VERIFY] All constraints satisfied" << std::endl;
        } else {
            std::cout << "[CONSTRAINT_VERIFY] Found " << violations.size() << " constraint violations" << std::endl;
        }
        
        return isValid;
    }
    
private:
    bool verifyNodeExistence(const std::vector<int>& path, std::vector<ConstraintViolation>& violations) const {
        bool allNodesExist = true;
        
        for (size_t i = 0; i < path.size(); ++i) {
            if (!graph->hasNode(path[i])) {
                violations.emplace_back("node_existence", 
                    "Node " + std::to_string(path[i]) + " does not exist in graph", 
                    static_cast<int>(i), 1.0);
                allNodesExist = false;
                
                if (strictMode) {
                    return false; // Fail immediately in strict mode
                }
            }
        }
        
        return allNodesExist;
    }
    
    bool verifyPathLength(const std::vector<int>& path, std::vector<ConstraintViolation>& violations) const {
        if (path.size() > maxPathLength) {
            violations.emplace_back("path_length", 
                "Path exceeds maximum allowed length of " + std::to_string(maxPathLength) + " nodes", 
                -1, static_cast<double>(path.size()) / maxPathLength);
            return false;
        }
        
        if (path.size() < 2) {
            violations.emplace_back("path_length", 
                "Path must contain at least 2 nodes for valid navigation", 
                -1, 1.0);
            return false;
        }
        
        return true;
    }
    
    bool verifySegmentLengths(const std::vector<int>& path, std::vector<ConstraintViolation>& violations) const {
        bool allSegmentsValid = true;
        
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasNode(path[i-1]) || !graph->hasNode(path[i])) {
                continue; // Skip if nodes don't exist (handled elsewhere)
            }
            
            const Node& from = graph->getNode(path[i-1]);
            const Node& to = graph->getNode(path[i]);
            double segmentLength = from.euclideanDistance(to);
            
            if (segmentLength > maxSegmentLength) {
                violations.emplace_back("segment_length", 
                    "Segment " + std::to_string(i-1) + "->" + std::to_string(i) + 
                    " exceeds maximum length (" + std::to_string(segmentLength) + " > " + 
                    std::to_string(maxSegmentLength) + ")", 
                    static_cast<int>(i-1), segmentLength / maxSegmentLength);
                allSegmentsValid = false;
            }
            
            if (segmentLength < 1e-6) {
                violations.emplace_back("segment_length", 
                    "Segment " + std::to_string(i-1) + "->" + std::to_string(i) + 
                    " has zero or near-zero length", 
                    static_cast<int>(i-1), 1.0);
                allSegmentsValid = false;
            }
        }
        
        return allSegmentsValid;
    }
    
    bool verifyTotalLength(const std::vector<int>& path, std::vector<ConstraintViolation>& violations) const {
        double totalLength = 0.0;
        
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasNode(path[i-1]) || !graph->hasNode(path[i])) {
                continue;
            }
            
            const Node& from = graph->getNode(path[i-1]);
            const Node& to = graph->getNode(path[i]);
            totalLength += from.euclideanDistance(to);
        }
        
        if (totalLength > maxTotalLength) {
            violations.emplace_back("total_length", 
                "Total path length (" + std::to_string(totalLength) + 
                ") exceeds maximum allowed (" + std::to_string(maxTotalLength) + ")", 
                -1, totalLength / maxTotalLength);
            return false;
        }
        
        return true;
    }
    
    bool verifyAngularConstraints(const std::vector<int>& path, std::vector<ConstraintViolation>& violations) const {
        if (path.size() < 3) return true; // No angles to check
        
        bool allAnglesValid = true;
        
        for (size_t i = 1; i < path.size() - 1; ++i) {
            if (!graph->hasNode(path[i-1]) || !graph->hasNode(path[i]) || !graph->hasNode(path[i+1])) {
                continue;
            }
            
            const Node& prev = graph->getNode(path[i-1]);
            const Node& curr = graph->getNode(path[i]);
            const Node& next = graph->getNode(path[i+1]);
            
            double angle = calculateAngle(prev, curr, next);
            double deviation = M_PI - angle;
            
            if (deviation > maxDeviationAngle) {
                violations.emplace_back("angular", 
                    "Sharp turn at node " + std::to_string(path[i]) + 
                    " (deviation: " + std::to_string(deviation) + " rad)", 
                    static_cast<int>(i), deviation / maxDeviationAngle);
                allAnglesValid = false;
            }
        }
        
        return allAnglesValid;
    }
    
    bool verifyConnectivityConstraints(const std::vector<int>& path, std::vector<ConstraintViolation>& violations) const {
        bool allConnected = true;
        
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasNode(path[i-1]) || !graph->hasNode(path[i])) {
                continue;
            }
            
            if (!graph->hasEdge(path[i-1], path[i])) {
                violations.emplace_back("connectivity", 
                    "No edge exists between nodes " + std::to_string(path[i-1]) + 
                    " and " + std::to_string(path[i]), 
                    static_cast<int>(i-1), 1.0);
                allConnected = false;
                
                if (strictMode) {
                    return false; // Fail immediately in strict mode
                }
            }
        }
        
        return allConnected;
    }
    
    bool verifyGeometricConstraints(const std::vector<int>& path, std::vector<ConstraintViolation>& violations) const {
        bool geometryValid = true;
        
        // Check for loops in path
        std::unordered_set<int> visitedNodes;
        for (size_t i = 0; i < path.size(); ++i) {
            if (visitedNodes.find(path[i]) != visitedNodes.end()) {
                violations.emplace_back("geometry", 
                    "Path contains loop - node " + std::to_string(path[i]) + " visited multiple times", 
                    static_cast<int>(i), 0.5);
                geometryValid = false;
                break; // One loop detection is enough
            }
            visitedNodes.insert(path[i]);
        }
        
        // Check for self-intersections (simplified)
        if (path.size() >= 4) {
            for (size_t i = 0; i < path.size() - 3; ++i) {
                for (size_t j = i + 2; j < path.size() - 1; ++j) {
                    if (checkSegmentIntersection(path, i, j)) {
                        violations.emplace_back("geometry", 
                            "Path segments intersect: segment " + std::to_string(i) + 
                            " and segment " + std::to_string(j), 
                            static_cast<int>(i), 0.7);
                        geometryValid = false;
                    }
                }
            }
        }
        
        return geometryValid;
    }
    
    double calculateAngle(const Node& prev, const Node& curr, const Node& next) const {
        double v1x = prev.getX() - curr.getX();
        double v1y = prev.getY() - curr.getY();
        double v2x = next.getX() - curr.getX();
        double v2y = next.getY() - curr.getY();
        
        double dot = v1x * v2x + v1y * v2y;
        double mag1 = std::sqrt(v1x * v1x + v1y * v1y);
        double mag2 = std::sqrt(v2x * v2x + v2y * v2y);
        
        if (mag1 == 0 || mag2 == 0) return M_PI;
        
        double cosAngle = dot / (mag1 * mag2);
        cosAngle = std::max(-1.0, std::min(1.0, cosAngle)); // Clamp to valid range
        
        return std::acos(cosAngle);
    }
    
    bool checkSegmentIntersection(const std::vector<int>& path, size_t seg1Start, size_t seg2Start) const {
        if (!graph->hasNode(path[seg1Start]) || !graph->hasNode(path[seg1Start + 1]) ||
            !graph->hasNode(path[seg2Start]) || !graph->hasNode(path[seg2Start + 1])) {
            return false;
        }
        
        const Node& p1 = graph->getNode(path[seg1Start]);
        const Node& p2 = graph->getNode(path[seg1Start + 1]);
        const Node& p3 = graph->getNode(path[seg2Start]);
        const Node& p4 = graph->getNode(path[seg2Start + 1]);
        
        // Check if segments share endpoints
        if (path[seg1Start] == path[seg2Start] || path[seg1Start] == path[seg2Start + 1] ||
            path[seg1Start + 1] == path[seg2Start] || path[seg1Start + 1] == path[seg2Start + 1]) {
            return false;
        }
        
        // Simple line intersection check
        return doLinesIntersect(p1.getX(), p1.getY(), p2.getX(), p2.getY(),
                               p3.getX(), p3.getY(), p4.getX(), p4.getY());
    }
    
    bool doLinesIntersect(double x1, double y1, double x2, double y2, 
                         double x3, double y3, double x4, double y4) const {
        double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (std::abs(denom) < 1e-10) return false; // Parallel lines
        
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
        
        return (t >= 0 && t <= 1 && u >= 0 && u <= 1);
    }
    
public:
    void setMaxSegmentLength(double maxLength) {
        maxSegmentLength = maxLength;
        std::cout << "[CONSTRAINT_VERIFY] Max segment length set to " << maxLength << std::endl;
    }
    
    void setMaxTotalLength(double maxLength) {
        maxTotalLength = maxLength;
        std::cout << "[CONSTRAINT_VERIFY] Max total length set to " << maxLength << std::endl;
    }
    
    void setMaxDeviationAngle(double maxAngle) {
        maxDeviationAngle = maxAngle;
        std::cout << "[CONSTRAINT_VERIFY] Max deviation angle set to " << maxAngle << " radians" << std::endl;
    }
    
    void setStrictMode(bool strict) {
        strictMode = strict;
        std::cout << "[CONSTRAINT_VERIFY] Strict mode " << (strict ? "enabled" : "disabled") << std::endl;
    }
};

// Global constraint verifier
static std::unique_ptr<ConstraintVerifier> g_constraintVerifier;

bool verifyPathConstraints(const Graph* graph, const std::vector<int>& path, 
                          std::string& violationReport, bool strictMode) {
    if (!g_constraintVerifier || g_constraintVerifier.get() == nullptr) {
        g_constraintVerifier = std::make_unique<ConstraintVerifier>(graph, strictMode);
    }
    
    std::vector<ConstraintVerifier::ConstraintViolation> violations;
    bool isValid = g_constraintVerifier->verifyPathConstraints(path, violations);
    
    // Generate violation report
    violationReport.clear();
    if (!violations.empty()) {
        violationReport = "Constraint Violations Found:\n";
        for (const auto& violation : violations) {
            violationReport += "- " + violation.type + ": " + violation.description + 
                              " (severity: " + std::to_string(violation.severity) + ")\n";
        }
    }
    
    return isValid;
}

bool checkPathStructuralConstraints(const Graph* graph, const std::vector<int>& path) {
    std::string report;
    return verifyPathConstraints(graph, path, report, false);
}

void setConstraintParameters(double maxSegmentLength, double maxTotalLength, double maxAngle) {
    if (g_constraintVerifier) {
        g_constraintVerifier->setMaxSegmentLength(maxSegmentLength);
        g_constraintVerifier->setMaxTotalLength(maxTotalLength);
        g_constraintVerifier->setMaxDeviationAngle(maxAngle);
    }
}
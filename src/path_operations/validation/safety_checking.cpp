#include "path_operations/PathValidator.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

class SafetyChecker {
private:
    const Graph* graph;
    
    struct SafetyMetrics {
        double overallSafetyScore;
        double collisionRisk;
        double isolationRisk;
        double environmentalRisk;
        double navigationRisk;
        std::vector<std::string> safetyWarnings;
        std::vector<int> dangerousNodes;
        std::vector<std::pair<int, int>> hazardousEdges;
        std::vector<int> emergencyExitPoints;
        
        SafetyMetrics() : overallSafetyScore(1.0), collisionRisk(0.0), isolationRisk(0.0),
                         environmentalRisk(0.0), navigationRisk(0.0) {}
    };
    
    struct SafetyZones {
        std::unordered_set<int> hazardZones;
        std::unordered_set<int> restrictedZones;
        std::unordered_set<int> emergencyZones;
        std::unordered_map<int, double> nodeSafetyRatings;
        std::unordered_map<std::string, double> edgeHazardLevels;
        
        SafetyZones() {}
    };
    
    SafetyZones safetyZones;
    double minimumSafetyScore;
    bool enableEmergencyPlanning;
    double hazardProximityThreshold;
    
public:
    SafetyChecker(const Graph* environment) 
        : graph(environment), minimumSafetyScore(0.6), 
          enableEmergencyPlanning(true), hazardProximityThreshold(5.0) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[SAFETY_CHECK] Safety checker initialized" << std::endl;
    }
    
    SafetyMetrics checkPathSafety(const std::vector<int>& path) {
        SafetyMetrics metrics;
        
        if (path.size() < 2) {
            metrics.safetyWarnings.push_back("Path too short for safety analysis");
            metrics.overallSafetyScore = 0.0;
            return metrics;
        }
        
        std::cout << "[SAFETY_CHECK] Checking safety for path with " << path.size() << " nodes" << std::endl;
        
        // Check for dangerous nodes and zones
        checkHazardousAreas(path, metrics);
        
        // Analyze collision risks
        analyzeCollisionRisks(path, metrics);
        
        // Check isolation and emergency exit availability
        checkIsolationRisks(path, metrics);
        
        // Evaluate environmental hazards
        evaluateEnvironmentalHazards(path, metrics);
        
        // Assess navigation safety
        assessNavigationSafety(path, metrics);
        
        // Plan emergency exits if enabled
        if (enableEmergencyPlanning) {
            planEmergencyExits(path, metrics);
        }
        
        // Calculate overall safety score
        calculateOverallSafety(metrics);
        
        std::cout << "[SAFETY_CHECK] Safety analysis complete - Overall score: " 
                  << metrics.overallSafetyScore << std::endl;
        
        return metrics;
    }
    
private:
    void checkHazardousAreas(const std::vector<int>& path, SafetyMetrics& metrics) {
        for (int nodeId : path) {
            // Check if node is in hazard zone
            if (safetyZones.hazardZones.find(nodeId) != safetyZones.hazardZones.end()) {
                metrics.dangerousNodes.push_back(nodeId);
                metrics.safetyWarnings.push_back("Path passes through hazard zone at node " + 
                                                std::to_string(nodeId));
            }
            
            // Check if node is in restricted zone
            if (safetyZones.restrictedZones.find(nodeId) != safetyZones.restrictedZones.end()) {
                metrics.safetyWarnings.push_back("Path enters restricted zone at node " + 
                                                std::to_string(nodeId));
            }
            
            // Check node safety rating
            auto safetyIt = safetyZones.nodeSafetyRatings.find(nodeId);
            if (safetyIt != safetyZones.nodeSafetyRatings.end() && safetyIt->second < 0.4) {
                metrics.dangerousNodes.push_back(nodeId);
                metrics.safetyWarnings.push_back("Node " + std::to_string(nodeId) + 
                                                " has low safety rating (" + 
                                                std::to_string(safetyIt->second) + ")");
            }
        }
        
        // Check edges for hazards
        for (size_t i = 1; i < path.size(); ++i) {
            std::string edgeKey = std::to_string(path[i-1]) + "_" + std::to_string(path[i]);
            auto hazardIt = safetyZones.edgeHazardLevels.find(edgeKey);
            if (hazardIt != safetyZones.edgeHazardLevels.end() && hazardIt->second > 0.6) {
                metrics.hazardousEdges.emplace_back(path[i-1], path[i]);
                metrics.safetyWarnings.push_back("Hazardous edge detected: " + edgeKey);
            }
        }
    }
    
    void analyzeCollisionRisks(const std::vector<int>& path, SafetyMetrics& metrics) {
        double totalCollisionRisk = 0.0;
        
        for (size_t i = 0; i < path.size(); ++i) {
            if (!graph->hasNode(path[i])) continue;
            
            // Check node congestion (high degree nodes have higher collision risk)
            std::vector<int> neighbors = graph->getNeighbors(path[i]);
            double congestionFactor = static_cast<double>(neighbors.size()) / 10.0; // Normalize
            
            // Check proximity to hazardous areas
            double proximityRisk = calculateHazardProximity(path[i]);
            
            // Calculate segment-specific collision risk
            double segmentRisk = std::min(1.0, congestionFactor + proximityRisk);
            totalCollisionRisk += segmentRisk;
            
            if (segmentRisk > 0.7) {
                metrics.safetyWarnings.push_back("High collision risk at node " + 
                                                std::to_string(path[i]));
            }
        }
        
        metrics.collisionRisk = totalCollisionRisk / path.size();
    }
    
    void checkIsolationRisks(const std::vector<int>& path, SafetyMetrics& metrics) {
        double isolationRisk = 0.0;
        
        for (int nodeId : path) {
            if (!graph->hasNode(nodeId)) continue;
            
            std::vector<int> neighbors = graph->getNeighbors(nodeId);
            
            // Single connection points are high isolation risk
            if (neighbors.size() <= 1) {
                isolationRisk += 0.8;
                metrics.safetyWarnings.push_back("Node " + std::to_string(nodeId) + 
                                                " is poorly connected (isolation risk)");
            } else if (neighbors.size() == 2) {
                isolationRisk += 0.3;
            }
            
            // Check if neighbors lead to dead ends
            int deadEndNeighbors = 0;
            for (int neighbor : neighbors) {
                if (graph->getNeighbors(neighbor).size() <= 1) {
                    deadEndNeighbors++;
                }
            }
            
            if (deadEndNeighbors > 0) {
                isolationRisk += deadEndNeighbors * 0.2;
                metrics.safetyWarnings.push_back("Node " + std::to_string(nodeId) + 
                                                " has " + std::to_string(deadEndNeighbors) + 
                                                " dead-end connections");
            }
        }
        
        metrics.isolationRisk = std::min(1.0, isolationRisk / path.size());
    }
    
    void evaluateEnvironmentalHazards(const std::vector<int>& path, SafetyMetrics& metrics) {
        double environmentalRisk = 0.0;
        
        // Check path through different terrain types
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasNode(path[i-1]) || !graph->hasNode(path[i])) continue;
            
            const Node& from = graph->getNode(path[i-1]);
            const Node& to = graph->getNode(path[i]);
            
            // Analyze terrain based on coordinates (simplified)
            double terrainRisk = calculateTerrainRisk(from, to);
            environmentalRisk += terrainRisk;
            
            // Check for environmental hazards
            if (terrainRisk > 0.7) {
                metrics.safetyWarnings.push_back("High environmental risk on segment " + 
                                                std::to_string(path[i-1]) + " -> " + 
                                                std::to_string(path[i]));
            }
        }
        
        metrics.environmentalRisk = environmentalRisk / (path.size() - 1);
    }
    
    void assessNavigationSafety(const std::vector<int>& path, SafetyMetrics& metrics) {
        double navigationRisk = 0.0;
        
        // Check for sharp turns and difficult navigation
        for (size_t i = 2; i < path.size(); ++i) {
            if (!graph->hasNode(path[i-2]) || !graph->hasNode(path[i-1]) || !graph->hasNode(path[i])) {
                continue;
            }
            
            const Node& prev = graph->getNode(path[i-2]);
            const Node& curr = graph->getNode(path[i-1]);
            const Node& next = graph->getNode(path[i]);
            
            double angle = calculateTurnAngle(prev, curr, next);
            double turnSharpness = (M_PI - angle) / M_PI;
            
            if (turnSharpness > 0.7) {
                navigationRisk += 0.5;
                metrics.safetyWarnings.push_back("Sharp turn at node " + std::to_string(path[i-1]));
            }
            
            // Check for rapid direction changes
            if (i > 2) {
                const Node& prevPrev = graph->getNode(path[i-3]);
                double prevAngle = calculateTurnAngle(prevPrev, prev, curr);
                
                if (std::abs(angle - prevAngle) > M_PI / 3) {
                    navigationRisk += 0.3;
                    metrics.safetyWarnings.push_back("Rapid direction change near node " + 
                                                    std::to_string(path[i-1]));
                }
            }
        }
        
        // Check segment lengths for safety
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasNode(path[i-1]) || !graph->hasNode(path[i])) continue;
            
            const Node& from = graph->getNode(path[i-1]);
            const Node& to = graph->getNode(path[i]);
            double distance = from.euclideanDistance(to);
            
            if (distance > 20.0) {
                navigationRisk += 0.2;
                metrics.safetyWarnings.push_back("Long segment detected: " + 
                                                std::to_string(path[i-1]) + " -> " + 
                                                std::to_string(path[i]));
            }
        }
        
        metrics.navigationRisk = std::min(1.0, navigationRisk / (path.size() - 1));
    }
    
    void planEmergencyExits(const std::vector<int>& path, SafetyMetrics& metrics) {
        // Find emergency zones near the path
        for (int nodeId : path) {
            // Check if node is an emergency zone
            if (safetyZones.emergencyZones.find(nodeId) != safetyZones.emergencyZones.end()) {
                metrics.emergencyExitPoints.push_back(nodeId);
                continue;
            }
            
            // Look for nearby emergency zones
            std::vector<int> neighbors = graph->getNeighbors(nodeId);
            for (int neighbor : neighbors) {
                if (safetyZones.emergencyZones.find(neighbor) != safetyZones.emergencyZones.end()) {
                    metrics.emergencyExitPoints.push_back(neighbor);
                    break;
                }
            }
        }
        
        // Check emergency exit spacing
        if (metrics.emergencyExitPoints.empty()) {
            metrics.safetyWarnings.push_back("No emergency exits available along path");
        } else if (metrics.emergencyExitPoints.size() < path.size() / 10) {
            metrics.safetyWarnings.push_back("Insufficient emergency exit coverage");
        }
    }
    
    void calculateOverallSafety(SafetyMetrics& metrics) {
        // Weighted combination of risk factors
        double riskWeights[] = {0.3, 0.25, 0.2, 0.25}; // collision, isolation, environmental, navigation
        double risks[] = {metrics.collisionRisk, metrics.isolationRisk, 
                         metrics.environmentalRisk, metrics.navigationRisk};
        
        double totalRisk = 0.0;
        for (int i = 0; i < 4; ++i) {
            totalRisk += risks[i] * riskWeights[i];
        }
        
        // Additional penalties
        double dangerPenalty = metrics.dangerousNodes.size() * 0.1;
        double hazardPenalty = metrics.hazardousEdges.size() * 0.08;
        double emergencyPenalty = metrics.emergencyExitPoints.empty() ? 0.15 : 0.0;
        
        totalRisk += dangerPenalty + hazardPenalty + emergencyPenalty;
        
        metrics.overallSafetyScore = std::max(0.0, 1.0 - totalRisk);
        
        // Add overall warning if score is too low
        if (metrics.overallSafetyScore < minimumSafetyScore) {
            metrics.safetyWarnings.push_back("Overall safety score below minimum threshold");
        }
    }
    
    double calculateHazardProximity(int nodeId) {
        if (!graph->hasNode(nodeId)) return 0.0;
        
        const Node& node = graph->getNode(nodeId);
        double proximityRisk = 0.0;
        
        // Check distance to all hazard zones
        for (int hazardNode : safetyZones.hazardZones) {
            if (!graph->hasNode(hazardNode)) continue;
            
            const Node& hazard = graph->getNode(hazardNode);
            double distance = node.euclideanDistance(hazard);
            
            if (distance <= hazardProximityThreshold) {
                proximityRisk += (hazardProximityThreshold - distance) / hazardProximityThreshold;
            }
        }
        
        return std::min(1.0, proximityRisk);
    }
    
    double calculateTerrainRisk(const Node& from, const Node& to) {
        // Simplified terrain risk based on coordinates
        double avgX = (from.getX() + to.getX()) / 2.0;
        double avgY = (from.getY() + to.getY()) / 2.0;
        
        // Simple heuristic: certain coordinate ranges are more dangerous
        double risk = 0.0;
        
        if (avgX < 0 || avgY < 0) risk += 0.2; // Negative coordinates = rough terrain
        if (std::abs(avgX - avgY) > 50) risk += 0.3; // Large coordinate differences = steep terrain
        if (avgX > 100 || avgY > 100) risk += 0.1; // Far coordinates = remote areas
        
        return std::min(1.0, risk);
    }
    
    double calculateTurnAngle(const Node& prev, const Node& curr, const Node& next) {
        double v1x = curr.getX() - prev.getX();
        double v1y = curr.getY() - prev.getY();
        double v2x = next.getX() - curr.getX();
        double v2y = next.getY() - curr.getY();
        
        double dot = v1x * v2x + v1y * v2y;
        double mag1 = std::sqrt(v1x * v1x + v1y * v1y);
        double mag2 = std::sqrt(v2x * v2x + v2y * v2y);
        
        if (mag1 == 0 || mag2 == 0) return M_PI;
        
        double cosAngle = dot / (mag1 * mag2);
        cosAngle = std::max(-1.0, std::min(1.0, cosAngle));
        
        return std::acos(cosAngle);
    }
    
public:
    void addHazardZone(int nodeId) {
        safetyZones.hazardZones.insert(nodeId);
        std::cout << "[SAFETY_CHECK] Added hazard zone at node " << nodeId << std::endl;
    }
    
    void addRestrictedZone(int nodeId) {
        safetyZones.restrictedZones.insert(nodeId);
        std::cout << "[SAFETY_CHECK] Added restricted zone at node " << nodeId << std::endl;
    }
    
    void addEmergencyZone(int nodeId) {
        safetyZones.emergencyZones.insert(nodeId);
        std::cout << "[SAFETY_CHECK] Added emergency zone at node " << nodeId << std::endl;
    }
    
    void setNodeSafetyRating(int nodeId, double rating) {
        safetyZones.nodeSafetyRatings[nodeId] = std::max(0.0, std::min(1.0, rating));
        std::cout << "[SAFETY_CHECK] Set safety rating for node " << nodeId << " to " << rating << std::endl;
    }
    
    void setEdgeHazardLevel(int fromId, int toId, double hazardLevel) {
        std::string edgeKey = std::to_string(fromId) + "_" + std::to_string(toId);
        safetyZones.edgeHazardLevels[edgeKey] = std::max(0.0, std::min(1.0, hazardLevel));
        std::cout << "[SAFETY_CHECK] Set hazard level for edge " << edgeKey << " to " << hazardLevel << std::endl;
    }
    
    void setMinimumSafetyScore(double score) {
        minimumSafetyScore = std::max(0.0, std::min(1.0, score));
        std::cout << "[SAFETY_CHECK] Set minimum safety score to " << score << std::endl;
    }
    
    void setHazardProximityThreshold(double threshold) {
        hazardProximityThreshold = threshold;
        std::cout << "[SAFETY_CHECK] Set hazard proximity threshold to " << threshold << std::endl;
    }
    
    void enableEmergencyPlanning(bool enable) {
        enableEmergencyPlanning = enable;
        std::cout << "[SAFETY_CHECK] Emergency planning " << (enable ? "enabled" : "disabled") << std::endl;
    }
};

// Global safety checker
static std::unique_ptr<SafetyChecker> g_safetyChecker;

bool checkPathSafety(const Graph* graph, const std::vector<int>& path, 
                    double& safetyScore, std::string& safetyReport) {
    if (!g_safetyChecker) {
        g_safetyChecker = std::make_unique<SafetyChecker>(graph);
    }
    
    auto metrics = g_safetyChecker->checkPathSafety(path);
    safetyScore = metrics.overallSafetyScore;
    
    // Generate safety report
    safetyReport = "Safety Analysis Report:\n";
    safetyReport += "Overall Safety Score: " + std::to_string(metrics.overallSafetyScore) + "\n";
    safetyReport += "Collision Risk: " + std::to_string(metrics.collisionRisk) + "\n";
    safetyReport += "Isolation Risk: " + std::to_string(metrics.isolationRisk) + "\n";
    safetyReport += "Environmental Risk: " + std::to_string(metrics.environmentalRisk) + "\n";
    safetyReport += "Navigation Risk: " + std::to_string(metrics.navigationRisk) + "\n";
    safetyReport += "Emergency Exits: " + std::to_string(metrics.emergencyExitPoints.size()) + "\n";
    
    if (!metrics.safetyWarnings.empty()) {
        safetyReport += "Safety Warnings:\n";
        for (const auto& warning : metrics.safetyWarnings) {
            safetyReport += "- " + warning + "\n";
        }
    }
    
    return metrics.overallSafetyScore >= 0.6; // Default safety threshold
}

void configureSafetyChecking(double minSafetyScore, double proximityThreshold, bool emergencyPlanning) {
    if (g_safetyChecker) {
        g_safetyChecker->setMinimumSafetyScore(minSafetyScore);
        g_safetyChecker->setHazardProximityThreshold(proximityThreshold);
        g_safetyChecker->enableEmergencyPlanning(emergencyPlanning);
    }
}
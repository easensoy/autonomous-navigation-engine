#include "path_operations/CostCalculator.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_map>
#include <vector>

class MultiCriteriaCostAnalyzer {
private:
    const Graph* graph;
    
    struct CriteriaSet {
        double distance;
        double time;
        double energy;
        double safety;
        double comfort;
        double reliability;
        double environmental;
        double monetary;
        
        CriteriaSet() : distance(0.0), time(0.0), energy(0.0), safety(0.0),
                       comfort(0.0), reliability(0.0), environmental(0.0), monetary(0.0) {}
        
        std::vector<double> toVector() const {
            return {distance, time, energy, safety, comfort, reliability, environmental, monetary};
        }
        
        void fromVector(const std::vector<double>& values) {
            if (values.size() >= 8) {
                distance = values[0];
                time = values[1];
                energy = values[2];
                safety = values[3];
                comfort = values[4];
                reliability = values[5];
                environmental = values[6];
                monetary = values[7];
            }
        }
    };
    
    struct WeightProfile {
        std::string name;
        CriteriaSet weights;
        std::string description;
        bool isActive;
        
        WeightProfile(const std::string& profileName) 
            : name(profileName), description(""), isActive(false) {}
    };
    
    struct MultiCriteriaResult {
        std::vector<int> path;
        CriteriaSet rawScores;
        CriteriaSet normalizedScores;
        double weightedScore;
        double paretoDominanceRank;
        std::string dominanceExplanation;
        std::vector<std::string> tradeoffs;
        
        MultiCriteriaResult() : weightedScore(0.0), paretoDominanceRank(0.0) {}
    };
    
    struct ParetoAnalysis {
        std::vector<MultiCriteriaResult> paretoOptimal;
        std::vector<MultiCriteriaResult> dominated;
        std::string analysisReport;
        double diversityIndex;
        
        ParetoAnalysis() : diversityIndex(0.0) {}
    };
    
    std::vector<WeightProfile> weightProfiles;
    WeightProfile currentProfile;
    CriteriaSet criteriaMaxValues;
    CriteriaSet criteriaMinValues;
    
    // Analysis configuration
    bool enableParetoAnalysis;
    bool enableSensitivityAnalysis;
    bool normalizeCriteria;
    double toleranceThreshold;
    
    CriteriaSet calculatePathCriteria(const std::vector<int>& path) const {
        CriteriaSet criteria;
        
        if (path.size() < 2) return criteria;
        
        std::cout << "[MULTI_CRITERIA] Calculating criteria for path with " << path.size() << " nodes" << std::endl;
        
        // Distance calculation
        criteria.distance = calculateTotalDistance(path);
        
        // Time estimation
        criteria.time = estimatePathTime(path);
        
        // Energy consumption
        criteria.energy = calculateEnergyConsumption(path);
        
        // Safety assessment
        criteria.safety = assessPathSafety(path);
        
        // Comfort evaluation
        criteria.comfort = evaluatePathComfort(path);
        
        // Reliability analysis
        criteria.reliability = analyzePathReliability(path);
        
        // Environmental impact
        criteria.environmental = calculateEnvironmentalImpact(path);
        
        // Monetary cost
        criteria.monetary = calculateMonetaryCost(path);
        
        return criteria;
    }
    
    double calculateTotalDistance(const std::vector<int>& path) const {
        double totalDistance = 0.0;
        
        for (size_t i = 1; i < path.size(); ++i) {
            const Node& from = graph->getNode(path[i-1]);
            const Node& to = graph->getNode(path[i]);
            totalDistance += from.euclideanDistance(to);
        }
        
        return totalDistance;
    }
    
    double estimatePathTime(const std::vector<int>& path) const {
        double baseSpeed = 1.0; // units per time
        double totalDistance = calculateTotalDistance(path);
        double baseTime = totalDistance / baseSpeed;
        
        // Apply speed modifiers based on path characteristics
        double complexityFactor = 1.0;
        for (size_t i = 0; i < path.size(); ++i) {
            std::vector<int> neighbors = graph->getNeighbors(path[i]);
            if (neighbors.size() > 4) {
                complexityFactor += 0.1; // Junctions slow down travel
            }
        }
        
        return baseTime * complexityFactor;
    }
    
    double calculateEnergyConsumption(const std::vector<int>& path) const {
        double baseEnergyRate = 0.1; // energy per distance unit
        double totalDistance = calculateTotalDistance(path);
        double baseEnergy = totalDistance * baseEnergyRate;
        
        // Add energy penalties for direction changes
        double directionChangePenalty = 0.0;
        for (size_t i = 2; i < path.size(); ++i) {
            const Node& prev = graph->getNode(path[i-2]);
            const Node& curr = graph->getNode(path[i-1]);
            const Node& next = graph->getNode(path[i]);
            
            double angle1 = std::atan2(curr.getY() - prev.getY(), curr.getX() - prev.getX());
            double angle2 = std::atan2(next.getY() - curr.getY(), next.getX() - curr.getX());
            double angleChange = std::abs(angle2 - angle1);
            
            if (angleChange > M_PI) angleChange = 2 * M_PI - angleChange;
            directionChangePenalty += angleChange * 0.05;
        }
        
        return baseEnergy + directionChangePenalty;
    }
    
    double assessPathSafety(const std::vector<int>& path) const {
        double baseSafety = 1.0;
        double safetyPenalty = 0.0;
        
        for (int nodeId : path) {
            std::vector<int> neighbors = graph->getNeighbors(nodeId);
            
            // Nodes with fewer connections are considered less safe (fewer escape routes)
            if (neighbors.size() < 2) {
                safetyPenalty += 0.2;
            } else if (neighbors.size() == 2) {
                safetyPenalty += 0.1;
            }
        }
        
        // Convert to safety score (higher is better)
        return std::max(0.0, baseSafety - safetyPenalty / path.size());
    }
    
    double evaluatePathComfort(const std::vector<int>& path) const {
        double baseComfort = 1.0;
        double comfortPenalty = 0.0;
        
        // Penalty for path complexity (too many turns)
        int directionChanges = 0;
        for (size_t i = 2; i < path.size(); ++i) {
            const Node& prev = graph->getNode(path[i-2]);
            const Node& curr = graph->getNode(path[i-1]);
            const Node& next = graph->getNode(path[i]);
            
            double angle1 = std::atan2(curr.getY() - prev.getY(), curr.getX() - prev.getX());
            double angle2 = std::atan2(next.getY() - curr.getY(), next.getX() - curr.getX());
            double angleChange = std::abs(angle2 - angle1);
            
            if (angleChange > M_PI / 4) { // Significant direction change
                directionChanges++;
            }
        }
        
        comfortPenalty = static_cast<double>(directionChanges) / path.size();
        return std::max(0.0, baseComfort - comfortPenalty);
    }
    
    double analyzePathReliability(const std::vector<int>& path) const {
        double baseReliability = 1.0;
        double reliabilityPenalty = 0.0;
        
        // Assess connectivity redundancy
        for (int nodeId : path) {
            std::vector<int> neighbors = graph->getNeighbors(nodeId);
            
            // Single points of failure reduce reliability
            if (neighbors.size() == 1) {
                reliabilityPenalty += 0.3;
            } else if (neighbors.size() == 2) {
                reliabilityPenalty += 0.1;
            }
        }
        
        return std::max(0.0, baseReliability - reliabilityPenalty / path.size());
    }
    
    double calculateEnvironmentalImpact(const std::vector<int>& path) const {
        // Lower values indicate better environmental performance
        double baseImpact = calculateTotalDistance(path) * 0.1; // Base carbon footprint
        
        // Add impact based on path characteristics
        double complexityImpact = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            std::vector<int> neighbors = graph->getNeighbors(path[i]);
            if (neighbors.size() > 3) {
                complexityImpact += 0.05; // Urban areas have higher impact
            }
        }
        
        return baseImpact + complexityImpact;
    }
    
    double calculateMonetaryCost(const std::vector<int>& path) const {
        double baseCostPerUnit = 1.0;
        double totalDistance = calculateTotalDistance(path);
        double baseCost = totalDistance * baseCostPerUnit;
        
        // Add complexity surcharges
        double complexitySurcharge = 0.0;
        for (int nodeId : path) {
            std::vector<int> neighbors = graph->getNeighbors(nodeId);
            if (neighbors.size() > 4) {
                complexitySurcharge += 0.5; // Premium for complex routing
            }
        }
        
        return baseCost + complexitySurcharge;
    }
    
    CriteriaSet normalizeCriteriaSet(const CriteriaSet& rawCriteria) const {
        if (!normalizeCriteria) return rawCriteria;
        
        CriteriaSet normalized;
        std::vector<double> rawValues = rawCriteria.toVector();
        std::vector<double> minValues = criteriaMinValues.toVector();
        std::vector<double> maxValues = criteriaMaxValues.toVector();
        
        for (size_t i = 0; i < rawValues.size(); ++i) {
            double range = maxValues[i] - minValues[i];
            if (range > 0.001) {
                double normalizedValue = (rawValues[i] - minValues[i]) / range;
                rawValues[i] = std::max(0.0, std::min(1.0, normalizedValue));
            }
        }
        
        normalized.fromVector(rawValues);
        return normalized;
    }
    
    double calculateWeightedScore(const CriteriaSet& normalizedCriteria) const {
        std::vector<double> criteria = normalizedCriteria.toVector();
        std::vector<double> weights = currentProfile.weights.toVector();
        
        double weightedSum = 0.0;
        for (size_t i = 0; i < criteria.size() && i < weights.size(); ++i) {
            // For criteria where lower is better (distance, time, energy, environmental, monetary),
            // we need to invert them for the weighted sum
            double adjustedCriteria = criteria[i];
            if (i == 0 || i == 1 || i == 2 || i == 6 || i == 7) { // distance, time, energy, environmental, monetary
                adjustedCriteria = 1.0 - criteria[i];
            }
            
            weightedSum += adjustedCriteria * weights[i];
        }
        
        return weightedSum;
    }
    
    bool isDominatedBy(const CriteriaSet& solution1, const CriteriaSet& solution2) const {
        std::vector<double> sol1 = solution1.toVector();
        std::vector<double> sol2 = solution2.toVector();
        
        bool strictlyWorse = false;
        for (size_t i = 0; i < sol1.size() && i < sol2.size(); ++i) {
            // For minimization criteria (distance, time, energy, environmental, monetary)
            if (i == 0 || i == 1 || i == 2 || i == 6 || i == 7) {
                if (sol1[i] < sol2[i]) return false; // sol1 is better in this criterion
                if (sol1[i] > sol2[i]) strictlyWorse = true;
            } else {
                // For maximization criteria (safety, comfort, reliability)
                if (sol1[i] > sol2[i]) return false; // sol1 is better in this criterion
                if (sol1[i] < sol2[i]) strictlyWorse = true;
            }
        }
        
        return strictlyWorse;
    }
    
    void updateCriteriaBounds(const CriteriaSet& criteria) {
        std::vector<double> values = criteria.toVector();
        std::vector<double> minValues = criteriaMinValues.toVector();
        std::vector<double> maxValues = criteriaMaxValues.toVector();
        
        for (size_t i = 0; i < values.size(); ++i) {
            minValues[i] = std::min(minValues[i], values[i]);
            maxValues[i] = std::max(maxValues[i], values[i]);
        }
        
        criteriaMinValues.fromVector(minValues);
        criteriaMaxValues.fromVector(maxValues);
    }
    
    std::vector<std::string> analyzeTradeoffs(const CriteriaSet& criteria) const {
        std::vector<std::string> tradeoffs;
        std::vector<double> values = criteria.toVector();
        std::vector<std::string> criteriaNames = {
            "distance", "time", "energy", "safety", "comfort", "reliability", "environmental", "monetary"
        };
        
        // Identify criteria with extreme values
        for (size_t i = 0; i < values.size(); ++i) {
            double normalizedValue = (values[i] - criteriaMinValues.toVector()[i]) / 
                                   (criteriaMaxValues.toVector()[i] - criteriaMinValues.toVector()[i]);
            
            if (normalizedValue > 0.8) {
                if (i == 0 || i == 1 || i == 2 || i == 6 || i == 7) {
                    tradeoffs.push_back("High " + criteriaNames[i] + " cost");
                } else {
                    tradeoffs.push_back("Excellent " + criteriaNames[i]);
                }
            } else if (normalizedValue < 0.2) {
                if (i == 0 || i == 1 || i == 2 || i == 6 || i == 7) {
                    tradeoffs.push_back("Low " + criteriaNames[i] + " cost");
                } else {
                    tradeoffs.push_back("Poor " + criteriaNames[i]);
                }
            }
        }
        
        return tradeoffs;
    }
    
public:
    MultiCriteriaCostAnalyzer(const Graph* environment) 
        : graph(environment), currentProfile("default"), enableParetoAnalysis(true),
          enableSensitivityAnalysis(false), normalizeCriteria(true), toleranceThreshold(0.01) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        // Initialize default weight profile
        currentProfile.weights.distance = 0.2;
        currentProfile.weights.time = 0.2;
        currentProfile.weights.energy = 0.15;
        currentProfile.weights.safety = 0.2;
        currentProfile.weights.comfort = 0.1;
        currentProfile.weights.reliability = 0.1;
        currentProfile.weights.environmental = 0.05;
        currentProfile.weights.monetary = 0.0;
        currentProfile.isActive = true;
        
        // Initialize criteria bounds
        std::vector<double> maxInitial(8, 0.0);
        std::vector<double> minInitial(8, std::numeric_limits<double>::infinity());
        criteriaMaxValues.fromVector(maxInitial);
        criteriaMinValues.fromVector(minInitial);
        
        std::cout << "[MULTI_CRITERIA] Multi-criteria cost analyzer initialized" << std::endl;
    }
    
    MultiCriteriaResult evaluatePath(const std::vector<int>& path) {
        MultiCriteriaResult result;
        result.path = path;
        
        std::cout << "[MULTI_CRITERIA] Evaluating path with " << path.size() << " nodes" << std::endl;
        
        // Calculate raw criteria scores
        result.rawScores = calculatePathCriteria(path);
        updateCriteriaBounds(result.rawScores);
        
        // Normalize criteria if enabled
        result.normalizedScores = normalizeCriteriaSet(result.rawScores);
        
        // Calculate weighted score
        result.weightedScore = calculateWeightedScore(result.normalizedScores);
        
        // Analyze tradeoffs
        result.tradeoffs = analyzeTradeoffs(result.rawScores);
        
        std::cout << "[MULTI_CRITERIA] Path evaluation complete - Weighted score: " << result.weightedScore << std::endl;
        
        return result;
    }
    
    std::vector<MultiCriteriaResult> evaluateAlternativePaths(const std::vector<std::vector<int>>& pathAlternatives) {
        std::vector<MultiCriteriaResult> results;
        
        std::cout << "[MULTI_CRITERIA] Evaluating " << pathAlternatives.size() << " alternative paths" << std::endl;
        
        for (const auto& path : pathAlternatives) {
            MultiCriteriaResult result = evaluatePath(path);
            results.push_back(result);
        }
        
        // Sort by weighted score (descending)
        std::sort(results.begin(), results.end(),
            [](const MultiCriteriaResult& a, const MultiCriteriaResult& b) {
                return a.weightedScore > b.weightedScore;
            });
        
        return results;
    }
    
    ParetoAnalysis performParetoAnalysis(const std::vector<MultiCriteriaResult>& alternatives) {
        ParetoAnalysis analysis;
        
        if (!enableParetoAnalysis) {
            std::cout << "[MULTI_CRITERIA] Pareto analysis disabled" << std::endl;
            return analysis;
        }
        
        std::cout << "[MULTI_CRITERIA] Performing Pareto optimality analysis" << std::endl;
        
        for (size_t i = 0; i < alternatives.size(); ++i) {
            bool isDominated = false;
            
            for (size_t j = 0; j < alternatives.size(); ++j) {
                if (i != j && isDominatedBy(alternatives[i].normalizedScores, alternatives[j].normalizedScores)) {
                    isDominated = true;
                    break;
                }
            }
            
            if (isDominated) {
                analysis.dominated.push_back(alternatives[i]);
            } else {
                analysis.paretoOptimal.push_back(alternatives[i]);
            }
        }
        
        // Calculate diversity index
        if (analysis.paretoOptimal.size() > 1) {
            double diversitySum = 0.0;
            for (size_t i = 0; i < analysis.paretoOptimal.size(); ++i) {
                for (size_t j = i + 1; j < analysis.paretoOptimal.size(); ++j) {
                    std::vector<double> sol1 = analysis.paretoOptimal[i].normalizedScores.toVector();
                    std::vector<double> sol2 = analysis.paretoOptimal[j].normalizedScores.toVector();
                    
                    double distance = 0.0;
                    for (size_t k = 0; k < sol1.size(); ++k) {
                        distance += std::pow(sol1[k] - sol2[k], 2);
                    }
                    diversitySum += std::sqrt(distance);
                }
            }
            analysis.diversityIndex = diversitySum / (analysis.paretoOptimal.size() * (analysis.paretoOptimal.size() - 1) / 2);
        }
        
        // Generate analysis report
        analysis.analysisReport = "Pareto Analysis Results:\n";
        analysis.analysisReport += "Pareto optimal solutions: " + std::to_string(analysis.paretoOptimal.size()) + "\n";
        analysis.analysisReport += "Dominated solutions: " + std::to_string(analysis.dominated.size()) + "\n";
        analysis.analysisReport += "Diversity index: " + std::to_string(analysis.diversityIndex);
        
        std::cout << "[MULTI_CRITERIA] Pareto analysis complete - " << analysis.paretoOptimal.size() 
                  << " optimal, " << analysis.dominated.size() << " dominated" << std::endl;
        
        return analysis;
    }
    
    void addWeightProfile(const std::string& name, const CriteriaSet& weights, const std::string& description = "") {
        WeightProfile profile(name);
        profile.weights = weights;
        profile.description = description;
        
        weightProfiles.push_back(profile);
        
        std::cout << "[MULTI_CRITERIA] Added weight profile: " << name << std::endl;
    }
    
    bool setActiveProfile(const std::string& profileName) {
        for (auto& profile : weightProfiles) {
            if (profile.name == profileName) {
                currentProfile = profile;
                currentProfile.isActive = true;
                std::cout << "[MULTI_CRITERIA] Activated weight profile: " << profileName << std::endl;
                return true;
            }
        }
        
        std::cout << "[MULTI_CRITERIA] Weight profile not found: " << profileName << std::endl;
        return false;
    }
    
    void setManualWeights(double distance, double time, double energy, double safety,
                         double comfort, double reliability, double environmental, double monetary) {
        currentProfile.weights.distance = distance;
        currentProfile.weights.time = time;
        currentProfile.weights.energy = energy;
        currentProfile.weights.safety = safety;
        currentProfile.weights.comfort = comfort;
        currentProfile.weights.reliability = reliability;
        currentProfile.weights.environmental = environmental;
        currentProfile.weights.monetary = monetary;
        
        std::cout << "[MULTI_CRITERIA] Manual weights applied" << std::endl;
    }
    
    void enableParetoOptimization(bool enable) {
        enableParetoAnalysis = enable;
        std::cout << "[MULTI_CRITERIA] Pareto analysis " << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void enableNormalization(bool enable) {
        normalizeCriteria = enable;
        std::cout << "[MULTI_CRITERIA] Criteria normalization " << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void printCurrentProfile() const {
        std::cout << "[MULTI_CRITERIA] Current Weight Profile (" << currentProfile.name << "):" << std::endl;
        std::cout << "[MULTI_CRITERIA]   Distance: " << currentProfile.weights.distance << std::endl;
        std::cout << "[MULTI_CRITERIA]   Time: " << currentProfile.weights.time << std::endl;
        std::cout << "[MULTI_CRITERIA]   Energy: " << currentProfile.weights.energy << std::endl;
        std::cout << "[MULTI_CRITERIA]   Safety: " << currentProfile.weights.safety << std::endl;
        std::cout << "[MULTI_CRITERIA]   Comfort: " << currentProfile.weights.comfort << std::endl;
        std::cout << "[MULTI_CRITERIA]   Reliability: " << currentProfile.weights.reliability << std::endl;
        std::cout << "[MULTI_CRITERIA]   Environmental: " << currentProfile.weights.environmental << std::endl;
        std::cout << "[MULTI_CRITERIA]   Monetary: " << currentProfile.weights.monetary << std::endl;
    }
};
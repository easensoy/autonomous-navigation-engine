#include "navigation_strategies/LocalPathPlanner.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <unordered_set>

class PotentialFieldMethod {
private:
    const Graph* graph;
    
    struct PotentialField {
        std::unordered_map<int, double> attractivePotential;
        std::unordered_map<int, double> repulsivePotential;
        std::unordered_map<int, double> totalPotential;
        
        void clear() {
            attractivePotential.clear();
            repulsivePotential.clear();
            totalPotential.clear();
        }
    };
    
    struct ForceVector {
        double x, y;
        double magnitude;
        
        ForceVector(double xVal = 0.0, double yVal = 0.0) 
            : x(xVal), y(yVal), magnitude(std::sqrt(x*x + y*y)) {}
        
        ForceVector operator+(const ForceVector& other) const {
            return ForceVector(x + other.x, y + other.y);
        }
        
        ForceVector operator*(double scalar) const {
            return ForceVector(x * scalar, y * scalar);
        }
        
        void normalize() {
            if (magnitude > 0.001) {
                x /= magnitude;
                y /= magnitude;
                magnitude = 1.0;
            }
        }
    };
    
    // Configuration parameters
    double attractiveGain;
    double repulsiveGain;
    double influenceRadius;
    double goalThreshold;
    double stepSize;
    double maxForce;
    bool useQuadraticAttraction;
    bool enableLocalMinEscape;
    
    PotentialField currentField;
    std::vector<ForceVector> forceHistory;
    
    double calculateAttractivePotential(const Node& current, const Node& goal) const {
        double distance = current.euclideanDistance(goal);
        
        if (useQuadraticAttraction) {
            return 0.5 * attractiveGain * distance * distance;
        } else {
            return attractiveGain * distance;
        }
    }
    
    ForceVector calculateAttractiveForce(const Node& current, const Node& goal) const {
        double dx = goal.getX() - current.getX();
        double dy = goal.getY() - current.getY();
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < goalThreshold) {
            return ForceVector(0.0, 0.0);
        }
        
        if (distance < 0.001) distance = 0.001; // Avoid division by zero
        
        if (useQuadraticAttraction) {
            return ForceVector(attractiveGain * dx, attractiveGain * dy);
        } else {
            return ForceVector(attractiveGain * dx / distance, attractiveGain * dy / distance);
        }
    }
    
    double calculateRepulsivePotential(const Node& current, const Node& obstacle) const {
        double distance = current.euclideanDistance(obstacle);
        
        if (distance > influenceRadius) {
            return 0.0;
        }
        
        if (distance < 0.001) distance = 0.001; // Avoid singularity
        
        double eta = 1.0 / distance - 1.0 / influenceRadius;
        return 0.5 * repulsiveGain * eta * eta;
    }
    
    ForceVector calculateRepulsiveForce(const Node& current, const Node& obstacle) const {
        double dx = current.getX() - obstacle.getX();
        double dy = current.getY() - obstacle.getY();
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance > influenceRadius) {
            return ForceVector(0.0, 0.0);
        }
        
        if (distance < 0.001) distance = 0.001; // Avoid division by zero
        
        double eta = 1.0 / distance - 1.0 / influenceRadius;
        double forceMagnitude = repulsiveGain * eta / (distance * distance * distance);
        
        return ForceVector(forceMagnitude * dx, forceMagnitude * dy);
    }
    
    void computePotentialField(int goalId, const std::vector<int>& obstacles) {
        currentField.clear();
        
        const Node& goal = graph->getNode(goalId);
        
        // Compute potential for all nodes
        for (int nodeId : graph->getAllNodeIds()) {
            const Node& node = graph->getNode(nodeId);
            
            // Attractive potential
            double attractivePot = calculateAttractivePotential(node, goal);
            currentField.attractivePotential[nodeId] = attractivePot;
            
            // Repulsive potential
            double repulsivePot = 0.0;
            for (int obstacleId : obstacles) {
                const Node& obstacle = graph->getNode(obstacleId);
                repulsivePot += calculateRepulsivePotential(node, obstacle);
            }
            currentField.repulsivePotential[nodeId] = repulsivePot;
            
            // Total potential
            currentField.totalPotential[nodeId] = attractivePot + repulsivePot;
        }
        
        std::cout << "[POTENTIAL_FIELD] Computed potential field for " 
                  << graph->getNodeCount() << " nodes" << std::endl;
    }
    
    ForceVector calculateTotalForce(int nodeId, int goalId, const std::vector<int>& obstacles) const {
        const Node& current = graph->getNode(nodeId);
        const Node& goal = graph->getNode(goalId);
        
        // Attractive force
        ForceVector attractiveForce = calculateAttractiveForce(current, goal);
        
        // Repulsive forces
        ForceVector totalRepulsiveForce;
        for (int obstacleId : obstacles) {
            const Node& obstacle = graph->getNode(obstacleId);
            ForceVector repulsiveForce = calculateRepulsiveForce(current, obstacle);
            totalRepulsiveForce = totalRepulsiveForce + repulsiveForce;
        }
        
        // Combined force
        ForceVector totalForce = attractiveForce + totalRepulsiveForce;
        
        // Limit maximum force
        if (totalForce.magnitude > maxForce) {
            totalForce.normalize();
            totalForce = totalForce * maxForce;
        }
        
        return totalForce;
    }
    
    int findNextNodeByForce(int currentNode, const ForceVector& force) const {
        const Node& current = graph->getNode(currentNode);
        
        // Calculate target position
        double targetX = current.getX() + force.x * stepSize;
        double targetY = current.getY() + force.y * stepSize;
        
        // Find neighbor closest to target direction
        std::vector<int> neighbors = graph->getNeighbors(currentNode);
        int bestNeighbor = -1;
        double bestScore = -1.0;
        
        for (int neighbor : neighbors) {
            const Node& neighborNode = graph->getNode(neighbor);
            
            // Vector from current to neighbor
            double neighborDx = neighborNode.getX() - current.getX();
            double neighborDy = neighborNode.getY() - current.getY();
            double neighborDistance = std::sqrt(neighborDx*neighborDx + neighborDy*neighborDy);
            
            if (neighborDistance > 0.001) {
                // Normalize neighbor direction
                neighborDx /= neighborDistance;
                neighborDy /= neighborDistance;
                
                // Calculate alignment with force direction
                ForceVector normalizedForce = force;
                if (force.magnitude > 0.001) {
                    normalizedForce.normalize();
                }
                
                double alignment = neighborDx * normalizedForce.x + neighborDy * normalizedForce.y;
                
                if (alignment > bestScore) {
                    bestScore = alignment;
                    bestNeighbor = neighbor;
                }
            }
        }
        
        return bestNeighbor;
    }
    
    int findNextNodeByPotential(int currentNode, int goalId) const {
        std::vector<int> neighbors = graph->getNeighbors(currentNode);
        int bestNeighbor = -1;
        double lowestPotential = std::numeric_limits<double>::infinity();
        
        for (int neighbor : neighbors) {
            auto it = currentField.totalPotential.find(neighbor);
            if (it != currentField.totalPotential.end()) {
                if (it->second < lowestPotential) {
                    lowestPotential = it->second;
                    bestNeighbor = neighbor;
                }
            }
        }
        
        return bestNeighbor;
    }
    
    bool isLocalMinimum(int nodeId) const {
        auto it = currentField.totalPotential.find(nodeId);
        if (it == currentField.totalPotential.end()) return false;
        
        double currentPotential = it->second;
        std::vector<int> neighbors = graph->getNeighbors(nodeId);
        
        for (int neighbor : neighbors) {
            auto neighborIt = currentField.totalPotential.find(neighbor);
            if (neighborIt != currentField.totalPotential.end()) {
                if (neighborIt->second < currentPotential) {
                    return false;
                }
            }
        }
        
        return true;
    }
    
    std::vector<int> escapeLocalMinimum(int currentNode, int goalId, const std::vector<int>& obstacles) {
        std::cout << "[POTENTIAL_FIELD] Escaping local minimum at node " << currentNode << std::endl;
        
        // Random walk to escape local minimum
        std::vector<int> neighbors = graph->getNeighbors(currentNode);
        if (neighbors.empty()) return {currentNode};
        
        // Choose a random neighbor that's not an obstacle
        std::vector<int> safeNeighbors;
        std::unordered_set<int> obstacleSet(obstacles.begin(), obstacles.end());
        
        for (int neighbor : neighbors) {
            if (obstacleSet.find(neighbor) == obstacleSet.end()) {
                safeNeighbors.push_back(neighbor);
            }
        }
        
        if (safeNeighbors.empty()) return {currentNode};
        
        // Select neighbor with lowest repulsive potential
        int bestEscape = safeNeighbors[0];
        double lowestRepulsive = std::numeric_limits<double>::infinity();
        
        for (int neighbor : safeNeighbors) {
            auto it = currentField.repulsivePotential.find(neighbor);
            if (it != currentField.repulsivePotential.end()) {
                if (it->second < lowestRepulsive) {
                    lowestRepulsive = it->second;
                    bestEscape = neighbor;
                }
            }
        }
        
        std::cout << "[POTENTIAL_FIELD] Escaping to node " << bestEscape << std::endl;
        return {currentNode, bestEscape};
    }
    
    void recordForceHistory(const ForceVector& force) {
        forceHistory.push_back(force);
        
        // Keep only recent history
        if (forceHistory.size() > 10) {
            forceHistory.erase(forceHistory.begin());
        }
    }
    
    bool isOscillating() const {
        if (forceHistory.size() < 4) return false;
        
        // Check for oscillation in force direction
        size_t n = forceHistory.size();
        const ForceVector& current = forceHistory[n-1];
        const ForceVector& prev2 = forceHistory[n-3];
        
        // If forces are pointing in opposite directions
        double dotProduct = current.x * prev2.x + current.y * prev2.y;
        return dotProduct < -0.5;
    }
    
public:
    PotentialFieldMethod(const Graph* environment) 
        : graph(environment), attractiveGain(1.0), repulsiveGain(10.0),
          influenceRadius(2.0), goalThreshold(0.5), stepSize(0.5),
          maxForce(5.0), useQuadraticAttraction(true), enableLocalMinEscape(true) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[POTENTIAL_FIELD] Potential field method initialized" << std::endl;
    }
    
    std::vector<int> planWithPotentialField(int startId, int goalId, const std::vector<int>& obstacles) {
        std::cout << "[POTENTIAL_FIELD] Planning path from " << startId 
                  << " to " << goalId << " with " << obstacles.size() << " obstacles" << std::endl;
        
        forceHistory.clear();
        computePotentialField(goalId, obstacles);
        
        std::vector<int> path;
        int current = startId;
        path.push_back(current);
        
        const int maxIterations = 100;
        int iteration = 0;
        
        while (current != goalId && iteration < maxIterations) {
            int nextNode = -1;
            
            // Check for local minimum
            if (enableLocalMinEscape && isLocalMinimum(current)) {
                std::vector<int> escapeSegment = escapeLocalMinimum(current, goalId, obstacles);
                if (escapeSegment.size() > 1) {
                    nextNode = escapeSegment[1];
                }
            } else {
                // Normal potential field navigation
                ForceVector totalForce = calculateTotalForce(current, goalId, obstacles);
                recordForceHistory(totalForce);
                
                // Check for oscillation
                if (isOscillating()) {
                    std::cout << "[POTENTIAL_FIELD] Oscillation detected, using potential gradient" << std::endl;
                    nextNode = findNextNodeByPotential(current, goalId);
                } else {
                    nextNode = findNextNodeByForce(current, totalForce);
                }
            }
            
            if (nextNode == -1 || nextNode == current) {
                std::cout << "[POTENTIAL_FIELD] No valid next node found" << std::endl;
                break;
            }
            
            path.push_back(nextNode);
            current = nextNode;
            iteration++;
        }
        
        if (current == goalId) {
            std::cout << "[POTENTIAL_FIELD] Goal reached in " << iteration << " iterations" << std::endl;
        } else {
            std::cout << "[POTENTIAL_FIELD] Failed to reach goal after " << iteration << " iterations" << std::endl;
        }
        
        return path;
    }
    
    double getPotentialValue(int nodeId) const {
        auto it = currentField.totalPotential.find(nodeId);
        return (it != currentField.totalPotential.end()) ? it->second : 0.0;
    }
    
    ForceVector getForceAtNode(int nodeId, int goalId, const std::vector<int>& obstacles) const {
        return calculateTotalForce(nodeId, goalId, obstacles);
    }
    
    void setAttractiveGain(double gain) {
        attractiveGain = gain;
        std::cout << "[POTENTIAL_FIELD] Attractive gain set to " << gain << std::endl;
    }
    
    void setRepulsiveGain(double gain) {
        repulsiveGain = gain;
        std::cout << "[POTENTIAL_FIELD] Repulsive gain set to " << gain << std::endl;
    }
    
    void setInfluenceRadius(double radius) {
        influenceRadius = radius;
        std::cout << "[POTENTIAL_FIELD] Influence radius set to " << radius << std::endl;
    }
    
    void setStepSize(double size) {
        stepSize = size;
        std::cout << "[POTENTIAL_FIELD] Step size set to " << size << std::endl;
    }
    
    void enableQuadraticAttraction(bool enable) {
        useQuadraticAttraction = enable;
        std::cout << "[POTENTIAL_FIELD] Quadratic attraction " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void enableLocalMinimumEscape(bool enable) {
        enableLocalMinEscape = enable;
        std::cout << "[POTENTIAL_FIELD] Local minimum escape " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void printFieldStatistics() const {
        if (currentField.totalPotential.empty()) {
            std::cout << "[POTENTIAL_FIELD] No field computed yet" << std::endl;
            return;
        }
        
        double minPotential = std::numeric_limits<double>::infinity();
        double maxPotential = -std::numeric_limits<double>::infinity();
        
        for (const auto& pair : currentField.totalPotential) {
            minPotential = std::min(minPotential, pair.second);
            maxPotential = std::max(maxPotential, pair.second);
        }
        
        std::cout << "[POTENTIAL_FIELD] Field Statistics:" << std::endl;
        std::cout << "[POTENTIAL_FIELD]   Nodes computed: " << currentField.totalPotential.size() << std::endl;
        std::cout << "[POTENTIAL_FIELD]   Min potential: " << minPotential << std::endl;
        std::cout << "[POTENTIAL_FIELD]   Max potential: " << maxPotential << std::endl;
        std::cout << "[POTENTIAL_FIELD]   Force history length: " << forceHistory.size() << std::endl;
    }
    
    void printConfiguration() const {
        std::cout << "[POTENTIAL_FIELD] Configuration:" << std::endl;
        std::cout << "[POTENTIAL_FIELD]   Attractive gain: " << attractiveGain << std::endl;
        std::cout << "[POTENTIAL_FIELD]   Repulsive gain: " << repulsiveGain << std::endl;
        std::cout << "[POTENTIAL_FIELD]   Influence radius: " << influenceRadius << std::endl;
        std::cout << "[POTENTIAL_FIELD]   Goal threshold: " << goalThreshold << std::endl;
        std::cout << "[POTENTIAL_FIELD]   Step size: " << stepSize << std::endl;
        std::cout << "[POTENTIAL_FIELD]   Max force: " << maxForce << std::endl;
        std::cout << "[POTENTIAL_FIELD]   Quadratic attraction: " << useQuadraticAttraction << std::endl;
        std::cout << "[POTENTIAL_FIELD]   Local min escape: " << enableLocalMinEscape << std::endl;
    }
};
#include "navigation_strategies/LocalPathPlanner.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

class DynamicWindowApproach {
private:
    const Graph* graph;
    
    struct VelocityCommand {
        double linearVel;
        double angularVel;
        double score;
        double obstacleDistance;
        double goalHeading;
        double velocity;
        
        VelocityCommand(double lin = 0.0, double ang = 0.0) 
            : linearVel(lin), angularVel(ang), score(0.0), 
              obstacleDistance(0.0), goalHeading(0.0), velocity(0.0) {}
    };
    
    struct DynamicWindow {
        double minLinearVel;
        double maxLinearVel;
        double minAngularVel;
        double maxAngularVel;
        
        DynamicWindow() : minLinearVel(0.0), maxLinearVel(2.0), 
                         minAngularVel(-1.0), maxAngularVel(1.0) {}
    };
    
    struct RobotState {
        double x, y, theta;
        double linearVel, angularVel;
        
        RobotState(double xPos = 0.0, double yPos = 0.0, double heading = 0.0)
            : x(xPos), y(yPos), theta(heading), linearVel(0.0), angularVel(0.0) {}
    };
    
    // Configuration parameters
    double maxLinearVel;
    double maxAngularVel;
    double maxLinearAccel;
    double maxAngularAccel;
    double timeHorizon;
    double timeStep;
    double robotRadius;
    double safetyMargin;
    
    // Scoring weights
    double obstacleWeight;
    double goalWeight;
    double velocityWeight;
    
    RobotState getCurrentRobotState(int currentNodeId) const {
        const Node& node = graph->getNode(currentNodeId);
        return RobotState(node.getX(), node.getY(), 0.0);
    }
    
    DynamicWindow calculateDynamicWindow(const RobotState& state) const {
        DynamicWindow window;
        
        // Velocity constraints
        window.minLinearVel = std::max(0.0, state.linearVel - maxLinearAccel * timeHorizon);
        window.maxLinearVel = std::min(maxLinearVel, state.linearVel + maxLinearAccel * timeHorizon);
        
        window.minAngularVel = std::max(-maxAngularVel, state.angularVel - maxAngularAccel * timeHorizon);
        window.maxAngularVel = std::min(maxAngularVel, state.angularVel + maxAngularAccel * timeHorizon);
        
        return window;
    }
    
    std::vector<VelocityCommand> generateVelocitySamples(const DynamicWindow& window) const {
        std::vector<VelocityCommand> samples;
        
        const int linearSamples = 10;
        const int angularSamples = 15;
        
        double linearStep = (window.maxLinearVel - window.minLinearVel) / linearSamples;
        double angularStep = (window.maxAngularVel - window.minAngularVel) / angularSamples;
        
        for (int i = 0; i <= linearSamples; ++i) {
            for (int j = 0; j <= angularSamples; ++j) {
                double linearVel = window.minLinearVel + i * linearStep;
                double angularVel = window.minAngularVel + j * angularStep;
                
                samples.emplace_back(linearVel, angularVel);
            }
        }
        
        std::cout << "[DWA] Generated " << samples.size() << " velocity samples" << std::endl;
        return samples;
    }
    
    std::vector<std::pair<double, double>> simulateTrajectory(const RobotState& state, 
                                                            const VelocityCommand& cmd) const {
        std::vector<std::pair<double, double>> trajectory;
        
        double x = state.x;
        double y = state.y;
        double theta = state.theta;
        
        for (double t = 0; t <= timeHorizon; t += timeStep) {
            trajectory.emplace_back(x, y);
            
            // Update position using kinematic model
            x += cmd.linearVel * std::cos(theta) * timeStep;
            y += cmd.linearVel * std::sin(theta) * timeStep;
            theta += cmd.angularVel * timeStep;
        }
        
        return trajectory;
    }
    
    double evaluateObstacleDistance(const std::vector<std::pair<double, double>>& trajectory,
                                  const std::vector<int>& obstacles) const {
        if (obstacles.empty()) return 1000.0; // No obstacles
        
        double minDistance = std::numeric_limits<double>::infinity();
        
        for (const auto& [trajX, trajY] : trajectory) {
            for (int obstacleId : obstacles) {
                const Node& obstacle = graph->getNode(obstacleId);
                double distance = std::sqrt(
                    std::pow(trajX - obstacle.getX(), 2) + 
                    std::pow(trajY - obstacle.getY(), 2)
                );
                
                minDistance = std::min(minDistance, distance);
                
                // Early termination if too close
                if (distance < robotRadius + safetyMargin) {
                    return 0.0;
                }
            }
        }
        
        return minDistance;
    }
    
    double evaluateGoalAlignment(const std::vector<std::pair<double, double>>& trajectory,
                                double goalX, double goalY) const {
        if (trajectory.empty()) return 0.0;
        
        // Use final trajectory point
        const auto& finalPoint = trajectory.back();
        
        double distanceToGoal = std::sqrt(
            std::pow(finalPoint.first - goalX, 2) + 
            std::pow(finalPoint.second - goalY, 2)
        );
        
        // Closer to goal = higher score
        return 1.0 / (1.0 + distanceToGoal);
    }
    
    double evaluateVelocity(const VelocityCommand& cmd) const {
        // Prefer higher linear velocities for efficiency
        return cmd.linearVel / maxLinearVel;
    }
    
    void scoreVelocityCommand(VelocityCommand& cmd, const RobotState& state,
                            double goalX, double goalY, const std::vector<int>& obstacles) const {
        std::vector<std::pair<double, double>> trajectory = simulateTrajectory(state, cmd);
        
        cmd.obstacleDistance = evaluateObstacleDistance(trajectory, obstacles);
        cmd.goalHeading = evaluateGoalAlignment(trajectory, goalX, goalY);
        cmd.velocity = evaluateVelocity(cmd);
        
        // Combined score
        cmd.score = obstacleWeight * cmd.obstacleDistance + 
                   goalWeight * cmd.goalHeading + 
                   velocityWeight * cmd.velocity;
    }
    
    std::vector<int> convertTrajectoryToPath(const std::vector<std::pair<double, double>>& trajectory) const {
        std::vector<int> path;
        
        for (const auto& [x, y] : trajectory) {
            int nearestNode = findNearestNode(x, y);
            if (nearestNode != -1 && (path.empty() || path.back() != nearestNode)) {
                path.push_back(nearestNode);
            }
        }
        
        return path;
    }
    
    int findNearestNode(double x, double y) const {
        int nearestId = -1;
        double minDistance = std::numeric_limits<double>::infinity();
        
        for (int nodeId : graph->getAllNodeIds()) {
            const Node& node = graph->getNode(nodeId);
            double distance = std::sqrt(
                std::pow(node.getX() - x, 2) + std::pow(node.getY() - y, 2)
            );
            
            if (distance < minDistance) {
                minDistance = distance;
                nearestId = nodeId;
            }
        }
        
        return nearestId;
    }
    
public:
    DynamicWindowApproach(const Graph* environment) 
        : graph(environment), maxLinearVel(2.0), maxAngularVel(1.0),
          maxLinearAccel(1.0), maxAngularAccel(2.0), timeHorizon(3.0),
          timeStep(0.1), robotRadius(0.5), safetyMargin(0.2),
          obstacleWeight(0.4), goalWeight(0.4), velocityWeight(0.2) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[DWA] Dynamic Window Approach initialized" << std::endl;
    }
    
    std::vector<int> planLocalPath(int currentPosition, int goalPosition, 
                                 const std::vector<int>& obstacles) {
        std::cout << "[DWA] Planning local path from " << currentPosition 
                  << " to " << goalPosition << " with " << obstacles.size() << " obstacles" << std::endl;
        
        RobotState currentState = getCurrentRobotState(currentPosition);
        const Node& goalNode = graph->getNode(goalPosition);
        
        // Calculate dynamic window
        DynamicWindow window = calculateDynamicWindow(currentState);
        
        // Generate velocity samples
        std::vector<VelocityCommand> velocitySamples = generateVelocitySamples(window);
        
        // Score each velocity command
        for (auto& cmd : velocitySamples) {
            scoreVelocityCommand(cmd, currentState, goalNode.getX(), goalNode.getY(), obstacles);
        }
        
        // Select best velocity command
        auto bestCmd = std::max_element(velocitySamples.begin(), velocitySamples.end(),
            [](const VelocityCommand& a, const VelocityCommand& b) {
                return a.score < b.score;
            });
        
        if (bestCmd == velocitySamples.end() || bestCmd->score <= 0.0) {
            std::cout << "[DWA] No valid velocity command found" << std::endl;
            return {};
        }
        
        std::cout << "[DWA] Selected velocity: linear=" << bestCmd->linearVel 
                  << ", angular=" << bestCmd->angularVel << ", score=" << bestCmd->score << std::endl;
        
        // Generate trajectory for best command
        std::vector<std::pair<double, double>> trajectory = simulateTrajectory(currentState, *bestCmd);
        
        // Convert trajectory to node path
        std::vector<int> localPath = convertTrajectoryToPath(trajectory);
        
        // Ensure path contains at least current position
        if (localPath.empty() || localPath[0] != currentPosition) {
            localPath.insert(localPath.begin(), currentPosition);
        }
        
        std::cout << "[DWA] Generated local path with " << localPath.size() << " nodes" << std::endl;
        return localPath;
    }
    
    void setVelocityLimits(double maxLinear, double maxAngular) {
        maxLinearVel = maxLinear;
        maxAngularVel = maxAngular;
        std::cout << "[DWA] Velocity limits set: linear=" << maxLinear 
                  << ", angular=" << maxAngular << std::endl;
    }
    
    void setAccelerationLimits(double maxLinearAccel, double maxAngularAccel) {
        this->maxLinearAccel = maxLinearAccel;
        this->maxAngularAccel = maxAngularAccel;
        std::cout << "[DWA] Acceleration limits set: linear=" << maxLinearAccel 
                  << ", angular=" << maxAngularAccel << std::endl;
    }
    
    void setTimeHorizon(double horizon) {
        timeHorizon = horizon;
        std::cout << "[DWA] Time horizon set to " << horizon << "s" << std::endl;
    }
    
    void setRobotRadius(double radius) {
        robotRadius = radius;
        std::cout << "[DWA] Robot radius set to " << radius << std::endl;
    }
    
    void setScoringWeights(double obstacle, double goal, double velocity) {
        obstacleWeight = obstacle;
        goalWeight = goal;
        velocityWeight = velocity;
        
        // Normalize weights
        double total = obstacle + goal + velocity;
        if (total > 0) {
            obstacleWeight /= total;
            goalWeight /= total;
            velocityWeight /= total;
        }
        
        std::cout << "[DWA] Scoring weights set: obstacle=" << obstacleWeight 
                  << ", goal=" << goalWeight << ", velocity=" << velocityWeight << std::endl;
    }
    
    bool isTrajectoryValid(const std::vector<std::pair<double, double>>& trajectory,
                          const std::vector<int>& obstacles) const {
        return evaluateObstacleDistance(trajectory, obstacles) > robotRadius + safetyMargin;
    }
    
    void printConfiguration() const {
        std::cout << "[DWA] Configuration:" << std::endl;
        std::cout << "[DWA]   Max linear velocity: " << maxLinearVel << std::endl;
        std::cout << "[DWA]   Max angular velocity: " << maxAngularVel << std::endl;
        std::cout << "[DWA]   Max linear acceleration: " << maxLinearAccel << std::endl;
        std::cout << "[DWA]   Max angular acceleration: " << maxAngularAccel << std::endl;
        std::cout << "[DWA]   Time horizon: " << timeHorizon << std::endl;
        std::cout << "[DWA]   Robot radius: " << robotRadius << std::endl;
        std::cout << "[DWA]   Safety margin: " << safetyMargin << std::endl;
    }
};
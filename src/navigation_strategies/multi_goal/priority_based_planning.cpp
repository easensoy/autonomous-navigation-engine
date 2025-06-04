#include "navigation_strategies/MultiGoalPlanner.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include <iostream>
#include <algorithm>
#include <queue>
#include <chrono>
#include <unordered_map>

class PriorityBasedPlanner {
private:
    const Graph* graph;
    std::unique_ptr<AStar> aStarPlanner;
    std::unique_ptr<Dijkstra> dijkstraPlanner;
    
    struct PriorityDestination {
        int nodeId;
        std::string name;
        double priority;
        double urgency;
        std::chrono::steady_clock::time_point deadline;
        std::chrono::duration<double> serviceTime;
        std::vector<int> prerequisites;
        bool isCompleted;
        bool isCritical;
        double resourceRequirement;
        
        PriorityDestination(int id, const std::string& destName, double prio)
            : nodeId(id), name(destName), priority(prio), urgency(1.0),
              deadline(std::chrono::steady_clock::now() + std::chrono::hours(24)),
              serviceTime(std::chrono::minutes(0)), isCompleted(false),
              isCritical(false), resourceRequirement(1.0) {}
        
        double calculateEffectivePriority() const {
            auto now = std::chrono::steady_clock::now();
            auto timeToDeadline = std::chrono::duration_cast<std::chrono::hours>(deadline - now);
            
            double timeFactor = 1.0;
            if (timeToDeadline.count() > 0) {
                timeFactor = 24.0 / timeToDeadline.count(); // Higher factor as deadline approaches
            } else {
                timeFactor = 10.0; // Overdue tasks get very high priority
            }
            
            double criticalMultiplier = isCritical ? 2.0 : 1.0;
            return priority * urgency * timeFactor * criticalMultiplier;
        }
        
        bool operator<(const PriorityDestination& other) const {
            return calculateEffectivePriority() < other.calculateEffectivePriority();
        }
    };
    
    struct PlanningResult {
        std::vector<int> route;
        std::vector<PriorityDestination> visitOrder;
        double totalCost;
        double totalTime;
        double priorityScore;
        int destinationsVisited;
        bool allCriticalVisited;
        
        PlanningResult() : totalCost(0.0), totalTime(0.0), priorityScore(0.0),
                          destinationsVisited(0), allCriticalVisited(false) {}
    };
    
    std::vector<PriorityDestination> priorityDestinations;
    double availableTime;
    double availableResources;
    bool enforceDeadlines;
    bool allowPartialCompletion;
    
    std::priority_queue<PriorityDestination> createPriorityQueue() {
        std::priority_queue<PriorityDestination> pq;
        
        for (const auto& dest : priorityDestinations) {
            if (!dest.isCompleted) {
                pq.push(dest);
            }
        }
        
        std::cout << "[PRIORITY_PLANNER] Created priority queue with " 
                  << pq.size() << " destinations" << std::endl;
        return pq;
    }
    
    bool arePrerequisitesMet(const PriorityDestination& destination, 
                           const std::vector<int>& completedDestinations) const {
        for (int prereq : destination.prerequisites) {
            if (std::find(completedDestinations.begin(), completedDestinations.end(), prereq) 
                == completedDestinations.end()) {
                return false;
            }
        }
        return true;
    }
    
    bool isWithinTimeConstraint(const PriorityDestination& destination, 
                              double currentTime) const {
        if (!enforceDeadlines) return true;
        
        auto now = std::chrono::steady_clock::now();
        auto deadline = destination.deadline;
        auto remainingTime = std::chrono::duration_cast<std::chrono::seconds>(deadline - now);
        
        return remainingTime.count() > currentTime;
    }
    
    bool hasAvailableResources(const PriorityDestination& destination) const {
        return destination.resourceRequirement <= availableResources;
    }
    
    PlanningResult planGreedyPriorityRoute(int startNode) {
        std::cout << "[PRIORITY_PLANNER] Planning greedy priority-based route from " 
                  << startNode << std::endl;
        
        PlanningResult result;
        std::vector<int> completedDestinations;
        std::priority_queue<PriorityDestination> priorityQueue = createPriorityQueue();
        
        int currentNode = startNode;
        result.route.push_back(currentNode);
        double currentTime = 0.0;
        double remainingResources = availableResources;
        
        while (!priorityQueue.empty() && currentTime < availableTime) {
            std::vector<PriorityDestination> candidateDestinations;
            
            // Extract all destinations from priority queue to evaluate
            while (!priorityQueue.empty()) {
                candidateDestinations.push_back(priorityQueue.top());
                priorityQueue.pop();
            }
            
            // Find the best feasible destination
            PriorityDestination* bestDestination = nullptr;
            std::vector<int> bestPath;
            double bestTravelTime = std::numeric_limits<double>::infinity();
            
            for (auto& candidate : candidateDestinations) {
                if (!arePrerequisitesMet(candidate, completedDestinations)) {
                    continue;
                }
                
                if (!hasAvailableResources(candidate)) {
                    continue;
                }
                
                std::vector<int> pathToCandidate = aStarPlanner->findPath(currentNode, candidate.nodeId);
                if (pathToCandidate.empty()) {
                    continue;
                }
                
                double travelTime = calculateTravelTime(pathToCandidate);
                double totalTimeRequired = travelTime + candidate.serviceTime.count();
                
                if (currentTime + totalTimeRequired > availableTime) {
                    if (!allowPartialCompletion) {
                        continue;
                    }
                }
                
                if (!isWithinTimeConstraint(candidate, currentTime + totalTimeRequired)) {
                    continue;
                }
                
                // This destination is feasible - check if it's the best so far
                if (bestDestination == nullptr || 
                    candidate.calculateEffectivePriority() > bestDestination->calculateEffectivePriority()) {
                    bestDestination = &candidate;
                    bestPath = pathToCandidate;
                    bestTravelTime = travelTime;
                }
            }
            
            if (bestDestination == nullptr) {
                std::cout << "[PRIORITY_PLANNER] No more feasible destinations" << std::endl;
                break;
            }
            
            // Visit the best destination
            for (size_t i = 1; i < bestPath.size(); ++i) {
                result.route.push_back(bestPath[i]);
            }
            
            currentNode = bestDestination->nodeId;
            currentTime += bestTravelTime + bestDestination->serviceTime.count();
            remainingResources -= bestDestination->resourceRequirement;
            
            bestDestination->isCompleted = true;
            completedDestinations.push_back(bestDestination->nodeId);
            result.visitOrder.push_back(*bestDestination);
            result.destinationsVisited++;
            result.priorityScore += bestDestination->calculateEffectivePriority();
            
            std::cout << "[PRIORITY_PLANNER] Visited destination " << bestDestination->name 
                      << " (priority: " << bestDestination->calculateEffectivePriority() << ")" << std::endl;
            
            // Re-add remaining destinations to priority queue
            for (const auto& candidate : candidateDestinations) {
                if (!candidate.isCompleted) {
                    priorityQueue.push(candidate);
                }
            }
        }
        
        result.totalTime = currentTime;
        result.totalCost = calculateRouteCost(result.route);
        result.allCriticalVisited = checkAllCriticalVisited();
        
        std::cout << "[PRIORITY_PLANNER] Greedy route completed: " << result.destinationsVisited 
                  << " destinations, priority score: " << result.priorityScore << std::endl;
        
        return result;
    }
    
    PlanningResult planTimeOptimizedRoute(int startNode) {
        std::cout << "[PRIORITY_PLANNER] Planning time-optimized priority route" << std::endl;
        
        // Sort destinations by deadline urgency
        std::vector<PriorityDestination> sortedByDeadline = priorityDestinations;
        std::sort(sortedByDeadline.begin(), sortedByDeadline.end(),
            [](const PriorityDestination& a, const PriorityDestination& b) {
                return a.deadline < b.deadline;
            });
        
        PlanningResult result;
        int currentNode = startNode;
        result.route.push_back(currentNode);
        double currentTime = 0.0;
        std::vector<int> completedDestinations;
        
        for (auto& destination : sortedByDeadline) {
            if (destination.isCompleted) continue;
            
            if (!arePrerequisitesMet(destination, completedDestinations)) {
                continue;
            }
            
            std::vector<int> pathToDestination = aStarPlanner->findPath(currentNode, destination.nodeId);
            if (pathToDestination.empty()) continue;
            
            double travelTime = calculateTravelTime(pathToDestination);
            double totalTimeRequired = travelTime + destination.serviceTime.count();
            
            if (currentTime + totalTimeRequired <= availableTime &&
                isWithinTimeConstraint(destination, currentTime + totalTimeRequired) &&
                hasAvailableResources(destination)) {
                
                // Visit this destination
                for (size_t i = 1; i < pathToDestination.size(); ++i) {
                    result.route.push_back(pathToDestination[i]);
                }
                
                currentNode = destination.nodeId;
                currentTime += totalTimeRequired;
                destination.isCompleted = true;
                completedDestinations.push_back(destination.nodeId);
                result.visitOrder.push_back(destination);
                result.destinationsVisited++;
                result.priorityScore += destination.calculateEffectivePriority();
            }
        }
        
        result.totalTime = currentTime;
        result.totalCost = calculateRouteCost(result.route);
        result.allCriticalVisited = checkAllCriticalVisited();
        
        return result;
    }
    
    PlanningResult planCriticalFirstRoute(int startNode) {
        std::cout << "[PRIORITY_PLANNER] Planning critical-first priority route" << std::endl;
        
        PlanningResult result;
        int currentNode = startNode;
        result.route.push_back(currentNode);
        double currentTime = 0.0;
        std::vector<int> completedDestinations;
        
        // First pass: visit all critical destinations
        for (auto& destination : priorityDestinations) {
            if (!destination.isCritical || destination.isCompleted) continue;
            
            if (!arePrerequisitesMet(destination, completedDestinations)) {
                continue;
            }
            
            std::vector<int> pathToDestination = aStarPlanner->findPath(currentNode, destination.nodeId);
            if (pathToDestination.empty()) continue;
            
            double travelTime = calculateTravelTime(pathToDestination);
            double totalTimeRequired = travelTime + destination.serviceTime.count();
            
            if (currentTime + totalTimeRequired <= availableTime &&
                hasAvailableResources(destination)) {
                
                for (size_t i = 1; i < pathToDestination.size(); ++i) {
                    result.route.push_back(pathToDestination[i]);
                }
                
                currentNode = destination.nodeId;
                currentTime += totalTimeRequired;
                destination.isCompleted = true;
                completedDestinations.push_back(destination.nodeId);
                result.visitOrder.push_back(destination);
                result.destinationsVisited++;
                result.priorityScore += destination.calculateEffectivePriority();
                
                std::cout << "[PRIORITY_PLANNER] Visited critical destination " << destination.name << std::endl;
            }
        }
        
        // Second pass: visit remaining destinations by priority
        PlanningResult remainingResult = planGreedyPriorityRoute(currentNode);
        
        // Merge results
        result.route.insert(result.route.end(), remainingResult.route.begin() + 1, remainingResult.route.end());
        result.visitOrder.insert(result.visitOrder.end(), remainingResult.visitOrder.begin(), remainingResult.visitOrder.end());
        result.destinationsVisited += remainingResult.destinationsVisited;
        result.priorityScore += remainingResult.priorityScore;
        result.totalTime = currentTime + remainingResult.totalTime;
        result.totalCost = calculateRouteCost(result.route);
        result.allCriticalVisited = checkAllCriticalVisited();
        
        return result;
    }
    
    double calculateTravelTime(const std::vector<int>& path) const {
        double distance = calculateRouteCost(path);
        return distance / 1.0; // Assume unit speed
    }
    
    double calculateRouteCost(const std::vector<int>& route) const {
        if (route.size() < 2) return 0.0;
        
        double totalCost = 0.0;
        for (size_t i = 1; i < route.size(); ++i) {
            const Node& from = graph->getNode(route[i-1]);
            const Node& to = graph->getNode(route[i]);
            totalCost += from.euclideanDistance(to);
        }
        return totalCost;
    }
    
    bool checkAllCriticalVisited() const {
        for (const auto& dest : priorityDestinations) {
            if (dest.isCritical && !dest.isCompleted) {
                return false;
            }
        }
        return true;
    }
    
    void resetDestinationStates() {
        for (auto& dest : priorityDestinations) {
            dest.isCompleted = false;
        }
    }
    
public:
    PriorityBasedPlanner(const Graph* environment) 
        : graph(environment), availableTime(480.0), availableResources(100.0),
          enforceDeadlines(true), allowPartialCompletion(true) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        aStarPlanner = std::make_unique<AStar>(graph);
        dijkstraPlanner = std::make_unique<Dijkstra>(graph);
        
        std::cout << "[PRIORITY_PLANNER] Priority-based planner initialized" << std::endl;
    }
    
    std::vector<int> planOptimalPriorityRoute(int startNode, const std::string& strategy = "greedy") {
        std::cout << "[PRIORITY_PLANNER] Planning optimal priority route from " << startNode 
                  << " using " << strategy << " strategy" << std::endl;
        
        resetDestinationStates();
        
        PlanningResult result;
        
        if (strategy == "greedy") {
            result = planGreedyPriorityRoute(startNode);
        } else if (strategy == "time_optimized") {
            result = planTimeOptimizedRoute(startNode);
        } else if (strategy == "critical_first") {
            result = planCriticalFirstRoute(startNode);
        } else {
            // Auto-select strategy based on destinations
            std::vector<PlanningResult> candidates;
            
            resetDestinationStates();
            candidates.push_back(planGreedyPriorityRoute(startNode));
            
            resetDestinationStates();
            candidates.push_back(planTimeOptimizedRoute(startNode));
            
            resetDestinationStates();
            candidates.push_back(planCriticalFirstRoute(startNode));
            
            // Select best result based on priority score and critical completion
            result = *std::max_element(candidates.begin(), candidates.end(),
                [](const PlanningResult& a, const PlanningResult& b) {
                    if (a.allCriticalVisited != b.allCriticalVisited) {
                        return a.allCriticalVisited < b.allCriticalVisited;
                    }
                    return a.priorityScore < b.priorityScore;
                });
        }
        
        std::cout << "[PRIORITY_PLANNER] Planned route visiting " << result.destinationsVisited 
                  << " destinations with priority score " << result.priorityScore << std::endl;
        
        return result.route;
    }
    
    void addPriorityDestination(int nodeId, const std::string& name, double priority, 
                              bool isCritical = false) {
        PriorityDestination dest(nodeId, name, priority);
        dest.isCritical = isCritical;
        priorityDestinations.push_back(dest);
        
        std::cout << "[PRIORITY_PLANNER] Added " << (isCritical ? "critical " : "") 
                  << "destination: " << name << " (priority: " << priority << ")" << std::endl;
    }
    
    void setDestinationDeadline(int nodeId, std::chrono::steady_clock::time_point deadline) {
        for (auto& dest : priorityDestinations) {
            if (dest.nodeId == nodeId) {
                dest.deadline = deadline;
                std::cout << "[PRIORITY_PLANNER] Set deadline for destination " << dest.name << std::endl;
                break;
            }
        }
    }
    
    void setDestinationServiceTime(int nodeId, std::chrono::duration<double> serviceTime) {
        for (auto& dest : priorityDestinations) {
            if (dest.nodeId == nodeId) {
                dest.serviceTime = serviceTime;
                std::cout << "[PRIORITY_PLANNER] Set service time " << serviceTime.count() 
                          << "s for destination " << dest.name << std::endl;
                break;
            }
        }
    }
    
    void setDestinationPrerequisites(int nodeId, const std::vector<int>& prerequisites) {
        for (auto& dest : priorityDestinations) {
            if (dest.nodeId == nodeId) {
                dest.prerequisites = prerequisites;
                std::cout << "[PRIORITY_PLANNER] Set " << prerequisites.size() 
                          << " prerequisites for destination " << dest.name << std::endl;
                break;
            }
        }
    }
    
    void updateDestinationUrgency(int nodeId, double urgency) {
        for (auto& dest : priorityDestinations) {
            if (dest.nodeId == nodeId) {
                dest.urgency = urgency;
                std::cout << "[PRIORITY_PLANNER] Updated urgency to " << urgency 
                          << " for destination " << dest.name << std::endl;
                break;
            }
        }
    }
    
    void setAvailableTime(double timeLimit) {
        availableTime = timeLimit;
        std::cout << "[PRIORITY_PLANNER] Available time set to " << timeLimit << std::endl;
    }
    
    void setAvailableResources(double resourceLimit) {
        availableResources = resourceLimit;
        std::cout << "[PRIORITY_PLANNER] Available resources set to " << resourceLimit << std::endl;
    }
    
    void enableDeadlineEnforcement(bool enforce) {
        enforceDeadlines = enforce;
        std::cout << "[PRIORITY_PLANNER] Deadline enforcement " 
                  << (enforce ? "enabled" : "disabled") << std::endl;
    }
    
    void allowPartialCompletion(bool allow) {
        allowPartialCompletion = allow;
        std::cout << "[PRIORITY_PLANNER] Partial completion " 
                  << (allow ? "allowed" : "not allowed") << std::endl;
    }
    
    void clearDestinations() {
        priorityDestinations.clear();
        std::cout << "[PRIORITY_PLANNER] Cleared all priority destinations" << std::endl;
    }
    
    void printPriorityStatistics() const {
        std::cout << "[PRIORITY_PLANNER] Priority Statistics:" << std::endl;
        std::cout << "[PRIORITY_PLANNER]   Total destinations: " << priorityDestinations.size() << std::endl;
        
        int criticalCount = 0;
        int completedCount = 0;
        double totalPriority = 0.0;
        
        for (const auto& dest : priorityDestinations) {
            if (dest.isCritical) criticalCount++;
            if (dest.isCompleted) completedCount++;
            totalPriority += dest.calculateEffectivePriority();
        }
        
        std::cout << "[PRIORITY_PLANNER]   Critical destinations: " << criticalCount << std::endl;
        std::cout << "[PRIORITY_PLANNER]   Completed destinations: " << completedCount << std::endl;
        std::cout << "[PRIORITY_PLANNER]   Total priority score: " << totalPriority << std::endl;
        std::cout << "[PRIORITY_PLANNER]   Available time: " << availableTime << std::endl;
        std::cout << "[PRIORITY_PLANNER]   Available resources: " << availableResources << std::endl;
    }
};
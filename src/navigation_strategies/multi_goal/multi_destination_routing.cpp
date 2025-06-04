#include "navigation_strategies/MultiGoalPlanner.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <unordered_set>
#include <queue>

class MultiDestinationRouter {
private:
    const Graph* graph;
    std::unique_ptr<AStar> aStarPlanner;
    std::unique_ptr<Dijkstra> dijkstraPlanner;
    
    struct RouteSegment {
        int fromNode;
        int toNode;
        std::vector<int> path;
        double cost;
        double estimatedTime;
        bool isRequired;
        
        RouteSegment(int from, int to) 
            : fromNode(from), toNode(to), cost(0.0), estimatedTime(0.0), isRequired(true) {}
    };
    
    struct RoutingResult {
        std::vector<int> completeRoute;
        std::vector<RouteSegment> segments;
        double totalCost;
        double totalTime;
        bool successful;
        std::string strategy;
        
        RoutingResult() : totalCost(0.0), totalTime(0.0), successful(false) {}
    };
    
    struct DestinationNode {
        int nodeId;
        std::string name;
        double priority;
        bool isRequired;
        double serviceTime;
        std::vector<int> dependencies;
        
        DestinationNode(int id, const std::string& nodeName, double prio = 1.0)
            : nodeId(id), name(nodeName), priority(prio), isRequired(true), serviceTime(0.0) {}
    };
    
    std::vector<DestinationNode> destinations;
    std::unordered_map<int, std::vector<int>> precomputedPaths;
    double defaultTravelSpeed;
    bool enablePathCaching;
    bool optimizeForTime;
    
    void precomputeDistanceMatrix(const std::vector<int>& nodeList) {
        std::cout << "[MULTI_DESTINATION] Precomputing distance matrix for " 
                  << nodeList.size() << " destinations" << std::endl;
        
        precomputedPaths.clear();
        
        for (size_t i = 0; i < nodeList.size(); ++i) {
            for (size_t j = 0; j < nodeList.size(); ++j) {
                if (i != j) {
                    int fromNode = nodeList[i];
                    int toNode = nodeList[j];
                    
                    std::vector<int> path = aStarPlanner->findPath(fromNode, toNode);
                    if (!path.empty()) {
                        std::string key = std::to_string(fromNode) + "_" + std::to_string(toNode);
                        precomputedPaths[key] = path;
                    }
                }
            }
        }
        
        std::cout << "[MULTI_DESTINATION] Precomputed " << precomputedPaths.size() 
                  << " path segments" << std::endl;
    }
    
    std::vector<int> getPrecomputedPath(int fromNode, int toNode) {
        if (!enablePathCaching) {
            return aStarPlanner->findPath(fromNode, toNode);
        }
        
        std::string key = std::to_string(fromNode) + "_" + std::to_string(toNode);
        auto it = precomputedPaths.find(key);
        
        if (it != precomputedPaths.end()) {
            return it->second;
        }
        
        // Compute and cache the path
        std::vector<int> path = aStarPlanner->findPath(fromNode, toNode);
        precomputedPaths[key] = path;
        return path;
    }
    
    double calculatePathCost(const std::vector<int>& path) const {
        if (path.size() < 2) return 0.0;
        
        double totalCost = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            const Node& from = graph->getNode(path[i-1]);
            const Node& to = graph->getNode(path[i]);
            totalCost += from.euclideanDistance(to);
        }
        return totalCost;
    }
    
    double calculateTravelTime(const std::vector<int>& path) const {
        double distance = calculatePathCost(path);
        return distance / defaultTravelSpeed;
    }
    
    RoutingResult planSequentialRoute(int startNode, const std::vector<int>& destinations) {
        std::cout << "[MULTI_DESTINATION] Planning sequential route through " 
                  << destinations.size() << " destinations" << std::endl;
        
        RoutingResult result;
        result.strategy = "Sequential";
        result.successful = true;
        
        int currentNode = startNode;
        result.completeRoute.push_back(currentNode);
        
        for (int destination : destinations) {
            std::vector<int> segmentPath = getPrecomputedPath(currentNode, destination);
            
            if (segmentPath.empty()) {
                std::cout << "[MULTI_DESTINATION] No path found from " << currentNode 
                          << " to " << destination << std::endl;
                result.successful = false;
                break;
            }
            
            RouteSegment segment(currentNode, destination);
            segment.path = segmentPath;
            segment.cost = calculatePathCost(segmentPath);
            segment.estimatedTime = calculateTravelTime(segmentPath);
            
            result.segments.push_back(segment);
            result.totalCost += segment.cost;
            result.totalTime += segment.estimatedTime;
            
            // Add segment to complete route (excluding first node to avoid duplication)
            for (size_t i = 1; i < segmentPath.size(); ++i) {
                result.completeRoute.push_back(segmentPath[i]);
            }
            
            currentNode = destination;
        }
        
        std::cout << "[MULTI_DESTINATION] Sequential route: " << result.totalCost 
                  << " cost, " << result.totalTime << " time" << std::endl;
        
        return result;
    }
    
    RoutingResult planRadialRoute(int centralHub, const std::vector<int>& destinations) {
        std::cout << "[MULTI_DESTINATION] Planning radial route from hub " 
                  << centralHub << " to " << destinations.size() << " destinations" << std::endl;
        
        RoutingResult result;
        result.strategy = "Radial";
        result.successful = true;
        
        // Visit each destination and return to hub
        for (int destination : destinations) {
            // Path from hub to destination
            std::vector<int> outboundPath = getPrecomputedPath(centralHub, destination);
            // Path from destination back to hub
            std::vector<int> returnPath = getPrecomputedPath(destination, centralHub);
            
            if (outboundPath.empty() || returnPath.empty()) {
                std::cout << "[MULTI_DESTINATION] No path found for radial route to " 
                          << destination << std::endl;
                result.successful = false;
                continue;
            }
            
            RouteSegment outboundSegment(centralHub, destination);
            outboundSegment.path = outboundPath;
            outboundSegment.cost = calculatePathCost(outboundPath);
            outboundSegment.estimatedTime = calculateTravelTime(outboundPath);
            
            RouteSegment returnSegment(destination, centralHub);
            returnSegment.path = returnPath;
            returnSegment.cost = calculatePathCost(returnPath);
            returnSegment.estimatedTime = calculateTravelTime(returnPath);
            
            result.segments.push_back(outboundSegment);
            result.segments.push_back(returnSegment);
            result.totalCost += outboundSegment.cost + returnSegment.cost;
            result.totalTime += outboundSegment.estimatedTime + returnSegment.estimatedTime;
        }
        
        std::cout << "[MULTI_DESTINATION] Radial route: " << result.totalCost 
                  << " cost, " << result.totalTime << " time" << std::endl;
        
        return result;
    }
    
    RoutingResult planClusteredRoute(int startNode, const std::vector<int>& destinations) {
        std::cout << "[MULTI_DESTINATION] Planning clustered route for " 
                  << destinations.size() << " destinations" << std::endl;
        
        // Group destinations into clusters based on proximity
        std::vector<std::vector<int>> clusters = createGeographicClusters(destinations, 3);
        
        RoutingResult result;
        result.strategy = "Clustered";
        result.successful = true;
        result.completeRoute.push_back(startNode);
        
        int currentNode = startNode;
        
        for (const auto& cluster : clusters) {
            std::cout << "[MULTI_DESTINATION] Processing cluster with " 
                      << cluster.size() << " destinations" << std::endl;
            
            // Find optimal order within cluster
            std::vector<int> clusterOrder = optimizeClusterOrder(currentNode, cluster);
            
            // Visit each destination in the cluster
            for (int destination : clusterOrder) {
                std::vector<int> segmentPath = getPrecomputedPath(currentNode, destination);
                
                if (segmentPath.empty()) {
                    result.successful = false;
                    continue;
                }
                
                RouteSegment segment(currentNode, destination);
                segment.path = segmentPath;
                segment.cost = calculatePathCost(segmentPath);
                segment.estimatedTime = calculateTravelTime(segmentPath);
                
                result.segments.push_back(segment);
                result.totalCost += segment.cost;
                result.totalTime += segment.estimatedTime;
                
                // Add to complete route
                for (size_t i = 1; i < segmentPath.size(); ++i) {
                    result.completeRoute.push_back(segmentPath[i]);
                }
                
                currentNode = destination;
            }
        }
        
        std::cout << "[MULTI_DESTINATION] Clustered route: " << result.totalCost 
                  << " cost, " << result.totalTime << " time" << std::endl;
        
        return result;
    }
    
    std::vector<std::vector<int>> createGeographicClusters(const std::vector<int>& destinations, 
                                                          int maxClusters) const {
        std::vector<std::vector<int>> clusters;
        
        if (destinations.size() <= static_cast<size_t>(maxClusters)) {
            // Each destination becomes its own cluster
            for (int dest : destinations) {
                clusters.push_back({dest});
            }
            return clusters;
        }
        
        // Simple clustering based on coordinates
        std::vector<int> remaining = destinations;
        
        while (!remaining.empty() && static_cast<int>(clusters.size()) < maxClusters) {
            std::vector<int> currentCluster;
            
            // Start with first remaining destination
            int seedNode = remaining[0];
            currentCluster.push_back(seedNode);
            remaining.erase(remaining.begin());
            
            // Add nearby destinations to this cluster
            const Node& seedPosition = graph->getNode(seedNode);
            const double clusterRadius = 5.0; // Configurable cluster radius
            
            auto it = remaining.begin();
            while (it != remaining.end()) {
                const Node& candidatePosition = graph->getNode(*it);
                double distance = seedPosition.euclideanDistance(candidatePosition);
                
                if (distance <= clusterRadius) {
                    currentCluster.push_back(*it);
                    it = remaining.erase(it);
                } else {
                    ++it;
                }
            }
            
            clusters.push_back(currentCluster);
        }
        
        // Add any remaining destinations to the last cluster
        if (!remaining.empty() && !clusters.empty()) {
            clusters.back().insert(clusters.back().end(), remaining.begin(), remaining.end());
        }
        
        return clusters;
    }
    
    std::vector<int> optimizeClusterOrder(int startNode, const std::vector<int>& clusterNodes) const {
        if (clusterNodes.size() <= 1) return clusterNodes;
        
        // Use nearest neighbor heuristic for small clusters
        std::vector<int> optimizedOrder;
        std::unordered_set<int> unvisited(clusterNodes.begin(), clusterNodes.end());
        
        int current = startNode;
        
        while (!unvisited.empty()) {
            int nearest = findNearestUnvisited(current, unvisited);
            optimizedOrder.push_back(nearest);
            unvisited.erase(nearest);
            current = nearest;
        }
        
        return optimizedOrder;
    }
    
    int findNearestUnvisited(int currentNode, const std::unordered_set<int>& unvisited) const {
        const Node& current = graph->getNode(currentNode);
        int nearest = -1;
        double minDistance = std::numeric_limits<double>::infinity();
        
        for (int candidate : unvisited) {
            const Node& candidateNode = graph->getNode(candidate);
            double distance = current.euclideanDistance(candidateNode);
            
            if (distance < minDistance) {
                minDistance = distance;
                nearest = candidate;
            }
        }
        
        return nearest;
    }
    
    bool validateDependencies(const std::vector<int>& route) const {
        std::unordered_set<int> visited;
        
        for (int nodeId : route) {
            visited.insert(nodeId);
            
            // Check if all dependencies are satisfied
            for (const auto& dest : destinations) {
                if (dest.nodeId == nodeId) {
                    for (int dependency : dest.dependencies) {
                        if (visited.find(dependency) == visited.end()) {
                            return false;
                        }
                    }
                    break;
                }
            }
        }
        
        return true;
    }
    
public:
    MultiDestinationRouter(const Graph* environment) 
        : graph(environment), defaultTravelSpeed(1.0), enablePathCaching(true), 
          optimizeForTime(false) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        aStarPlanner = std::make_unique<AStar>(graph);
        dijkstraPlanner = std::make_unique<Dijkstra>(graph);
        
        std::cout << "[MULTI_DESTINATION] Multi-destination router initialized" << std::endl;
    }
    
    std::vector<int> planMultiDestinationRoute(int startNode, const std::vector<int>& destinationNodes, 
                                             const std::string& strategy = "auto") {
        std::cout << "[MULTI_DESTINATION] Planning route from " << startNode 
                  << " through " << destinationNodes.size() << " destinations using " 
                  << strategy << " strategy" << std::endl;
        
        if (destinationNodes.empty()) {
            return {startNode};
        }
        
        if (enablePathCaching) {
            std::vector<int> allNodes = {startNode};
            allNodes.insert(allNodes.end(), destinationNodes.begin(), destinationNodes.end());
            precomputeDistanceMatrix(allNodes);
        }
        
        RoutingResult bestResult;
        
        if (strategy == "auto") {
            // Try multiple strategies and select the best
            std::vector<RoutingResult> results;
            
            results.push_back(planSequentialRoute(startNode, destinationNodes));
            results.push_back(planClusteredRoute(startNode, destinationNodes));
            
            // Select best result based on optimization criteria
            bestResult = *std::min_element(results.begin(), results.end(),
                [this](const RoutingResult& a, const RoutingResult& b) {
                    if (optimizeForTime) {
                        return a.totalTime < b.totalTime;
                    } else {
                        return a.totalCost < b.totalCost;
                    }
                });
                
        } else if (strategy == "sequential") {
            bestResult = planSequentialRoute(startNode, destinationNodes);
        } else if (strategy == "radial") {
            bestResult = planRadialRoute(startNode, destinationNodes);
        } else if (strategy == "clustered") {
            bestResult = planClusteredRoute(startNode, destinationNodes);
        } else {
            bestResult = planSequentialRoute(startNode, destinationNodes);
        }
        
        if (bestResult.successful && validateDependencies(bestResult.completeRoute)) {
            std::cout << "[MULTI_DESTINATION] Successfully planned " << bestResult.strategy 
                      << " route with " << bestResult.segments.size() << " segments" << std::endl;
            return bestResult.completeRoute;
        }
        
        std::cout << "[MULTI_DESTINATION] Failed to plan valid multi-destination route" << std::endl;
        return {};
    }
    
    void addDestination(int nodeId, const std::string& name, double priority = 1.0) {
        destinations.emplace_back(nodeId, name, priority);
        std::cout << "[MULTI_DESTINATION] Added destination: " << name 
                  << " (node " << nodeId << ", priority " << priority << ")" << std::endl;
    }
    
    void setDestinationDependencies(int nodeId, const std::vector<int>& dependencies) {
        for (auto& dest : destinations) {
            if (dest.nodeId == nodeId) {
                dest.dependencies = dependencies;
                std::cout << "[MULTI_DESTINATION] Set " << dependencies.size() 
                          << " dependencies for destination " << nodeId << std::endl;
                break;
            }
        }
    }
    
    void setTravelSpeed(double speed) {
        defaultTravelSpeed = speed;
        std::cout << "[MULTI_DESTINATION] Travel speed set to " << speed << std::endl;
    }
    
    void enableCaching(bool enable) {
        enablePathCaching = enable;
        if (!enable) {
            precomputedPaths.clear();
        }
        std::cout << "[MULTI_DESTINATION] Path caching " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void optimizeFor(const std::string& objective) {
        if (objective == "time") {
            optimizeForTime = true;
            std::cout << "[MULTI_DESTINATION] Optimizing for travel time" << std::endl;
        } else if (objective == "distance") {
            optimizeForTime = false;
            std::cout << "[MULTI_DESTINATION] Optimizing for travel distance" << std::endl;
        }
    }
    
    double estimateRouteTime(const std::vector<int>& route) const {
        return calculateTravelTime(route);
    }
    
    double estimateRouteCost(const std::vector<int>& route) const {
        return calculatePathCost(route);
    }
    
    void clearDestinations() {
        destinations.clear();
        precomputedPaths.clear();
        std::cout << "[MULTI_DESTINATION] Cleared all destinations and cached paths" << std::endl;
    }
    
    void printRoutingStatistics() const {
        std::cout << "[MULTI_DESTINATION] Routing Statistics:" << std::endl;
        std::cout << "[MULTI_DESTINATION]   Destinations defined: " << destinations.size() << std::endl;
        std::cout << "[MULTI_DESTINATION]   Cached paths: " << precomputedPaths.size() << std::endl;
        std::cout << "[MULTI_DESTINATION]   Default travel speed: " << defaultTravelSpeed << std::endl;
        std::cout << "[MULTI_DESTINATION]   Optimization target: " 
                  << (optimizeForTime ? "time" : "distance") << std::endl;
    }
};
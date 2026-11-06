#include "navigation_strategies/MultiGoalPlanner.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <memory>
#include <utility>

// MultiGoalPlanner orders the goals with nearest-neighbour on true shortest-path costs, improves
// the order with 2-opt (start fixed, open tour), then joins the legs into one route.

namespace {
using DistanceTable = std::unordered_map<int, std::unordered_map<int, double>>;

DistanceTable shortestDistances(const Graph& graph, const std::vector<int>& stops) {
    DistanceTable table;
    Dijkstra dijkstra(&graph);
    for (int from : stops) {
        const auto distances = dijkstra.findShortestDistances(from);
        for (int to : stops) {
            auto it = distances.find(to);
            table[from][to] = it == distances.end() ? std::numeric_limits<double>::infinity() : it->second;
        }
    }
    return table;
}

double orderCost(const std::vector<int>& order, const DistanceTable& table) {
    double cost = 0.0;
    for (size_t i = 1; i < order.size(); ++i) {
        cost += table.at(order[i - 1]).at(order[i]);
    }
    return cost;
}
}  // namespace

MultiGoalPlanner::MultiGoalPlanner(const Graph* graph)
    : environment(graph), optimizationStrategy(MultiGoalStrategy::NEAREST_NEIGHBOR) {
    if (!environment) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
    globalPathPlanner = std::make_unique<GlobalPathPlanner>(environment);
}

std::vector<int> MultiGoalPlanner::nearestNeighborTSP(int startId, std::vector<int> goals) {
    std::vector<int> stops{startId};
    stops.insert(stops.end(), goals.begin(), goals.end());
    const DistanceTable table = shortestDistances(*environment, stops);

    std::vector<int> order{startId};
    while (!goals.empty()) {
        auto nearest = std::min_element(goals.begin(), goals.end(), [&](int a, int b) {
            return table.at(order.back()).at(a) < table.at(order.back()).at(b);
        });
        order.push_back(*nearest);
        goals.erase(nearest);
    }

    // 2-opt: reverse any segment (never the fixed start) that shortens the open tour.
    bool improved = true;
    while (improved) {
        improved = false;
        for (size_t i = 1; i + 1 < order.size(); ++i) {
            for (size_t j = i + 1; j < order.size(); ++j) {
                std::vector<int> candidate = order;
                std::reverse(candidate.begin() + static_cast<long>(i), candidate.begin() + static_cast<long>(j) + 1);
                if (orderCost(candidate, table) + 1e-9 < orderCost(order, table)) {
                    order = std::move(candidate);
                    improved = true;
                }
            }
        }
    }
    return order;
}

double MultiGoalPlanner::calculateTotalDistance(const std::vector<int>& tour) {
    double total = 0.0;
    for (size_t i = 1; i < tour.size(); ++i) {
        if (!environment->hasEdge(tour[i - 1], tour[i])) {
            return std::numeric_limits<double>::infinity();
        }
        total += environment->getEdgeWeight(tour[i - 1], tour[i]);
    }
    return total;
}

std::vector<int> MultiGoalPlanner::getOptimalVisitationOrder(int startId, const std::vector<int>& goals) {
    if (!environment->hasNode(startId)) {
        return {};
    }
    std::vector<int> unique;
    for (int goal : goals) {
        if (goal != startId && environment->hasNode(goal) &&
            std::find(unique.begin(), unique.end(), goal) == unique.end()) {
            unique.push_back(goal);
        }
    }
    // The genetic, annealing and branch-and-bound strategies share this nearest-neighbour + 2-opt order.
    return nearestNeighborTSP(startId, unique);
}

std::vector<int> MultiGoalPlanner::planOptimalTour(int startId, const std::vector<int>& goals) {
    const std::vector<int> order = getOptimalVisitationOrder(startId, goals);
    if (order.empty()) {
        return {};
    }
    std::vector<int> route{startId};
    for (size_t i = 1; i < order.size(); ++i) {
        std::vector<int> leg = globalPathPlanner->planPath(order[i - 1], order[i]);
        if (leg.empty()) {
            return {};  // a goal cannot be reached
        }
        route.insert(route.end(), leg.begin() + 1, leg.end());
    }
    return route;
}

std::vector<int> MultiGoalPlanner::planOptimalTourWithReturn(int startId, const std::vector<int>& goals) {
    std::vector<int> route = planOptimalTour(startId, goals);
    if (route.size() < 2) {
        return route;
    }
    std::vector<int> back = globalPathPlanner->planPath(route.back(), startId);
    if (back.empty()) {
        return {};
    }
    route.insert(route.end(), back.begin() + 1, back.end());
    return route;
}

std::vector<std::vector<int>> MultiGoalPlanner::planIndividualPaths(int startId, const std::vector<int>& goals) {
    return globalPathPlanner->planMultipleGoals(startId, goals);
}

void MultiGoalPlanner::setOptimizationStrategy(MultiGoalStrategy strategy) {
    optimizationStrategy = strategy;
}

double MultiGoalPlanner::estimateTourCost(int startId, const std::vector<int>& goals) {
    const std::vector<int> route = planOptimalTour(startId, goals);
    return route.empty() ? std::numeric_limits<double>::infinity() : calculateTotalDistance(route);
}

#include "path_operations/CostCalculator.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <limits>
#include <numeric>
#include <stdexcept>

// CostCalculator prices a route from its edge weights, with optional per-edge overrides and
// per-node traversal costs. A route with a missing edge costs infinity.

CostCalculator::CostCalculator(const Graph* graph) : environment(graph) {
    if (!environment) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
    resetCostModel();
}

double CostCalculator::getSegmentCost(int fromId, int toId) const {
    auto custom = edgeCosts.find({fromId, toId});
    if (custom != edgeCosts.end()) {
        return custom->second;
    }
    if (!environment->hasEdge(fromId, toId)) {
        return std::numeric_limits<double>::infinity();
    }
    return environment->getEdgeWeight(fromId, toId);
}

double CostCalculator::getNodeTraversalCost(int nodeId) const {
    auto it = nodeCosts.find(nodeId);
    return it == nodeCosts.end() ? 0.0 : it->second;
}

double CostCalculator::calculatePathCost(const std::vector<int>& path) const {
    if (path.empty()) {
        return 0.0;
    }
    double cost = getNodeTraversalCost(path.front());
    for (size_t i = 1; i < path.size(); ++i) {
        cost += getSegmentCost(path[i - 1], path[i]) + getNodeTraversalCost(path[i]);
    }
    return cost * costWeights.at(CostType::DISTANCE);
}

double CostCalculator::calculatePathCostWithType(const std::vector<int>& path, CostType costType) const {
    const CostBreakdown breakdown = calculateDetailedCost(path);
    switch (costType) {
        case CostType::DISTANCE: return breakdown.distanceCost;
        case CostType::TIME: return breakdown.timeCost;
        case CostType::ENERGY: return breakdown.energyCost;
        case CostType::RISK: return breakdown.riskCost;
        case CostType::MONETARY: return 0.0;  // no monetary data in the graph
        case CostType::COMPOSITE: return breakdown.totalCost;
    }
    return breakdown.totalCost;
}

CostBreakdown CostCalculator::calculateDetailedCost(const std::vector<int>& path) const {
    CostBreakdown breakdown{};
    for (size_t i = 1; i < path.size(); ++i) {
        breakdown.distanceCost += getSegmentCost(path[i - 1], path[i]);
    }
    for (int node : path) {
        breakdown.riskCost += getNodeTraversalCost(node);
    }
    breakdown.timeCost = breakdown.distanceCost;    // unit speed: time follows distance
    breakdown.energyCost = breakdown.distanceCost;  // no slope or load data: energy follows distance
    breakdown.totalCost = costWeights.at(CostType::DISTANCE) * breakdown.distanceCost +
                          costWeights.at(CostType::TIME) * breakdown.timeCost +
                          costWeights.at(CostType::ENERGY) * breakdown.energyCost +
                          costWeights.at(CostType::RISK) * breakdown.riskCost;
    return breakdown;
}

void CostCalculator::setCostWeight(CostType costType, double weight) {
    if (weight < 0.0) {
        throw std::invalid_argument("Cost weights cannot be negative");
    }
    costWeights[costType] = weight;
}

void CostCalculator::setNodeCost(int nodeId, double cost) {
    nodeCosts[nodeId] = cost;
}

void CostCalculator::setEdgeCost(int fromId, int toId, double cost) {
    edgeCosts[{fromId, toId}] = cost;
}

double CostCalculator::estimateCostToGoal(int currentId, int goalId) const {
    if (!environment->hasNode(currentId) || !environment->hasNode(goalId)) {
        return std::numeric_limits<double>::infinity();
    }
    const Node& a = environment->getNode(currentId);
    const Node& b = environment->getNode(goalId);
    return std::hypot(a.getX() - b.getX(), a.getY() - b.getY());  // straight line, coordinate units
}

void CostCalculator::resetCostModel() {
    costWeights = {{CostType::DISTANCE, 1.0}, {CostType::TIME, 0.0},     {CostType::ENERGY, 0.0},
                   {CostType::RISK, 0.0},     {CostType::MONETARY, 0.0}, {CostType::COMPOSITE, 0.0}};
    nodeCosts.clear();
    edgeCosts.clear();
}

std::vector<double> CostCalculator::calculateCostProfile(const std::vector<int>& path) const {
    std::vector<double> profile;
    for (size_t i = 1; i < path.size(); ++i) {
        profile.push_back(getSegmentCost(path[i - 1], path[i]) + getNodeTraversalCost(path[i]));
    }
    return profile;
}

int CostCalculator::findCostliestSegment(const std::vector<int>& path) const {
    const std::vector<double> profile = calculateCostProfile(path);
    if (profile.empty()) {
        return -1;
    }
    return static_cast<int>(std::max_element(profile.begin(), profile.end()) - profile.begin());
}

double CostCalculator::calculateCostVariance(const std::vector<int>& path) const {
    const std::vector<double> profile = calculateCostProfile(path);
    if (profile.empty()) {
        return 0.0;
    }
    const double mean = std::accumulate(profile.begin(), profile.end(), 0.0) / static_cast<double>(profile.size());
    double variance = 0.0;
    for (double value : profile) {
        variance += (value - mean) * (value - mean);
    }
    return variance / static_cast<double>(profile.size());
}

namespace {
const std::pair<CostType, const char*> kCostTypeNames[] = {
    {CostType::DISTANCE, "DISTANCE"}, {CostType::TIME, "TIME"}, {CostType::ENERGY, "ENERGY"},
    {CostType::RISK, "RISK"}, {CostType::MONETARY, "MONETARY"}, {CostType::COMPOSITE, "COMPOSITE"}};

std::string modelError(const std::string& file, int line, const std::string& what) {
    return "CostCalculator::loadCostModel: " + file + ":" + std::to_string(line) + ": " + what;
}
}  // namespace

void CostCalculator::saveCostModel(const std::string& modelFile) const {
    std::ofstream out(modelFile, std::ios::binary);
    if (!out) {
        throw std::runtime_error("CostCalculator::saveCostModel: cannot write " + modelFile);
    }
    out.precision(std::numeric_limits<double>::max_digits10);
    out << "cost-model 1\n";
    for (const auto& [type, name] : kCostTypeNames) {
        const auto weight = costWeights.find(type);
        out << "weight " << name << " " << (weight == costWeights.end() ? 0.0 : weight->second) << "\n";
    }
    std::map<int, double> nodes(nodeCosts.begin(), nodeCosts.end());
    for (const auto& [node, cost] : nodes) {
        out << "node " << node << " " << cost << "\n";
    }
    std::map<std::pair<int, int>, double> edges(edgeCosts.begin(), edgeCosts.end());
    for (const auto& [edge, cost] : edges) {
        out << "edge " << edge.first << " " << edge.second << " " << cost << "\n";
    }
    if (!out.flush()) {
        throw std::runtime_error("CostCalculator::saveCostModel: cannot write " + modelFile);
    }
}

void CostCalculator::loadCostModel(const std::string& modelFile) {
    std::ifstream in(modelFile, std::ios::binary);
    if (!in) {
        throw std::runtime_error("CostCalculator::loadCostModel: cannot read " + modelFile);
    }
    // Parse into a fresh model so that a malformed file leaves the current one untouched.
    CostCalculator loaded(environment);
    std::string line;
    int lineNumber = 0;
    bool sawHeader = false;
    while (std::getline(in, line)) {
        ++lineNumber;
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty()) {
            continue;
        }
        std::istringstream fields(line);
        std::string keyword;
        fields >> keyword;
        if (!sawHeader) {
            int version = 0;
            if (keyword != "cost-model" || !(fields >> version) || version != 1) {
                throw std::runtime_error(modelError(modelFile, lineNumber, "expected the header 'cost-model 1'"));
            }
            sawHeader = true;
        } else if (keyword == "weight") {
            std::string name;
            double value = 0.0;
            if (!(fields >> name >> value) || !std::isfinite(value) || value < 0.0) {
                throw std::runtime_error(
                    modelError(modelFile, lineNumber, "expected 'weight <TYPE> <non-negative value>'"));
            }
            const auto type = std::find_if(std::begin(kCostTypeNames), std::end(kCostTypeNames),
                                           [&name](const auto& entry) { return name == entry.second; });
            if (type == std::end(kCostTypeNames)) {
                throw std::runtime_error(modelError(modelFile, lineNumber, "unknown cost type '" + name + "'"));
            }
            loaded.costWeights[type->first] = value;
        } else if (keyword == "node") {
            int node = 0;
            double cost = 0.0;
            if (!(fields >> node >> cost) || !std::isfinite(cost)) {
                throw std::runtime_error(modelError(modelFile, lineNumber, "expected 'node <id> <cost>'"));
            }
            loaded.nodeCosts[node] = cost;
        } else if (keyword == "edge") {
            int from = 0;
            int to = 0;
            double cost = 0.0;
            if (!(fields >> from >> to >> cost) || !std::isfinite(cost)) {
                throw std::runtime_error(modelError(modelFile, lineNumber, "expected 'edge <from> <to> <cost>'"));
            }
            loaded.edgeCosts[{from, to}] = cost;
        } else {
            throw std::runtime_error(modelError(modelFile, lineNumber, "unknown entry '" + keyword + "'"));
        }
        std::string extra;
        if (fields >> extra) {
            throw std::runtime_error(modelError(modelFile, lineNumber, "unexpected text '" + extra + "'"));
        }
    }
    if (!sawHeader) {
        throw std::runtime_error(modelError(modelFile, lineNumber, "empty file"));
    }
    costWeights = std::move(loaded.costWeights);
    nodeCosts = std::move(loaded.nodeCosts);
    edgeCosts = std::move(loaded.edgeCosts);
}

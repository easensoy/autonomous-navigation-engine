#include "core/Graph.hpp"
#include "graph_operations/ConnectivityAnalyzer.hpp"
#include "graph_operations/EdgeManager.hpp"
#include "graph_operations/GraphBuilder.hpp"
#include "graph_operations/NodeManager.hpp"
#include "path_operations/CostCalculator.hpp"
#include "path_operations/PathSmoother.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Behaviour of the members implemented in November 2026 (CHANGELOG.md): GraphBuilder,
// NodeManager::moveNode and renameNode, EdgeManager::getMinimumSpanningTree,
// ConnectivityAnalyzer::getBridgeEdges, CostCalculator::saveCostModel and loadCostModel, and
// PathSmoother::preserveCriticalWaypoints. Results are checked against independent references:
// brute force for bridges, the cycle property for spanning trees, exact round trips for the model.

namespace {

class CoutSilencer {
public:
    CoutSilencer() : saved(std::cout.rdbuf(nullptr)) {}
    ~CoutSilencer() {
        std::cout.rdbuf(saved);
        std::cout.clear();
    }

private:
    std::streambuf* saved;
};

template <typename Exception, typename Call>
bool throws(Call call) {
    try {
        CoutSilencer quiet;
        call();
    } catch (const Exception&) {
        return true;
    }
    return false;
}

// Undirected pairs (smaller id first) joined by an edge in either direction.
std::set<std::pair<int, int>> undirectedPairs(const Graph& graph) {
    std::set<std::pair<int, int>> pairs;
    for (int id : graph.getAllNodeIds()) {
        for (const Edge& edge : graph.getEdgesFrom(id)) {
            if (edge.getToNode() != id) {
                pairs.insert({std::min(id, edge.getToNode()), std::max(id, edge.getToNode())});
            }
        }
    }
    return pairs;
}

int componentCount(const Graph& graph) {
    ConnectivityAnalyzer analyzer(&graph);
    return static_cast<int>(analyzer.findConnectedComponents().size());
}

double distance(const Graph& graph, int a, int b) {
    const Node& p = graph.getNode(a);
    const Node& q = graph.getNode(b);
    return std::hypot(p.getX() - q.getX(), p.getY() - q.getY());
}

// Random graph for the reference checks; raw mt19937 output keeps it identical on every compiler.
Graph randomGraph(std::mt19937& rng, int nodes, int edges) {
    CoutSilencer quiet;
    Graph graph;
    for (int i = 0; i < nodes; ++i) {
        graph.addNode(i, "n", static_cast<double>(rng() % 100), static_cast<double>(rng() % 100));
    }
    for (int e = 0; e < edges; ++e) {
        const int a = static_cast<int>(rng() % static_cast<unsigned>(nodes));
        const int b = static_cast<int>(rng() % static_cast<unsigned>(nodes));
        if (a != b && !graph.hasEdge(a, b) && !graph.hasEdge(b, a)) {
            graph.addEdge(a, b, 1.0 + static_cast<double>(rng() % 20), rng() % 4 != 0);
        }
    }
    return graph;
}

}  // namespace

class GraphOperationsApiTester {
private:
    int testsRun = 0;
    int testsPassed = 0;

    void runTest(const std::string& name, std::function<bool()> test) {
        testsRun++;
        std::cout << "[TEST] " << name << ": ";
        bool passed = false;
        try {
            passed = test();
        } catch (const std::exception& e) {
            std::cout << "EXCEPTION: " << e.what() << " ";
        }
        if (passed) {
            testsPassed++;
        }
        std::cout << (passed ? "PASS" : "FAIL") << std::endl;
    }

public:
    void runAllTests() {
        std::cout << "\n=== Graph Operations API Tests ===" << std::endl;

        runTest("GraphBuilder rejects a null graph", []() {
            return throws<std::invalid_argument>([] { GraphBuilder builder(nullptr); });
        });

        runTest("GRID builds width x height nodes joined orthogonally by edges of weight spacing", []() {
            Graph graph;
            GraphBuilder builder(&graph);
            {
                CoutSilencer quiet;
                builder.buildGraph(GraphType::GRID, {{"width", 4}, {"height", 3}, {"spacing", 2.5}});
            }
            bool positions = graph.getNodeCount() == 12;
            for (int y = 0; y < 3 && positions; ++y) {
                for (int x = 0; x < 4; ++x) {
                    const Node& node = graph.getNode(y * 4 + x);
                    positions = positions && node.getX() == x * 2.5 && node.getY() == y * 2.5;
                }
            }
            const auto pairs = undirectedPairs(graph);
            bool edges = pairs.size() == 17;  // 3 x 3 horizontal + 4 x 2 vertical
            for (const auto& [a, b] : pairs) {
                const bool orthogonal = (b - a == 1 && a / 4 == b / 4) || b - a == 4;
                edges = edges && orthogonal && graph.getEdgeWeight(a, b) == 2.5 && graph.hasEdge(b, a);
            }
            return positions && edges;
        });

        runTest("buildGraph rejects a non-empty graph and missing, unknown or out-of-range parameters", []() {
            Graph used;
            {
                CoutSilencer quiet;
                used.addNode(0, "n", 0, 0);
            }
            Graph empty;
            GraphBuilder onUsed(&used);
            GraphBuilder onEmpty(&empty);
            return throws<std::invalid_argument>([&] { onUsed.buildGraph(GraphType::GRID, {{"width", 2}, {"height", 2}}); }) &&
                   throws<std::invalid_argument>([&] { onEmpty.buildGraph(GraphType::GRID, {{"width", 2}}); }) &&
                   throws<std::invalid_argument>([&] { onEmpty.buildGraph(GraphType::GRID, {{"width", 2}, {"height", 2}, {"depth", 1}}); }) &&
                   throws<std::invalid_argument>([&] { onEmpty.buildGraph(GraphType::GRID, {{"width", 2.5}, {"height", 2}}); }) &&
                   throws<std::invalid_argument>([&] { onEmpty.buildGraph(GraphType::GRID, {{"width", 2}, {"height", 2}, {"spacing", 0}}); }) &&
                   throws<std::invalid_argument>([&] { onEmpty.buildGraph(GraphType::RANDOM, {{"nodeCount", 5}, {"connectionProbability", 1.5}}); }) &&
                   empty.isEmpty();
        });

        runTest("RANDOM places n nodes, weights edges by distance, and repeats for a fixed seed", []() {
            auto build = [](double probability, double seed) {
                Graph graph;
                GraphBuilder builder(&graph);
                CoutSilencer quiet;
                builder.buildGraph(GraphType::RANDOM, {{"nodeCount", 30}, {"connectionProbability", probability}, {"seed", seed}});
                return graph;
            };
            const Graph first = build(0.2, 7);
            const Graph second = build(0.2, 7);
            bool valid = first.getNodeCount() == 30;
            for (int id = 0; id < 30 && valid; ++id) {
                const Node& node = first.getNode(id);
                valid = node.getX() >= 0 && node.getX() < 100 && node.getY() >= 0 && node.getY() < 100 &&
                        node.getX() == second.getNode(id).getX() && node.getY() == second.getNode(id).getY();
            }
            const auto pairs = undirectedPairs(first);
            for (const auto& [a, b] : pairs) {
                valid = valid && first.hasEdge(b, a) && std::fabs(first.getEdgeWeight(a, b) - distance(first, a, b)) < 1e-9;
            }
            return valid && pairs == undirectedPairs(second) && undirectedPairs(build(0.0, 1)).empty() &&
                   undirectedPairs(build(1.0, 1)).size() == 30 * 29 / 2;
        });

        runTest("addNodesFromCoordinates numbers new nodes after the largest id; bad input adds nothing", []() {
            Graph graph;
            {
                CoutSilencer quiet;
                graph.addNode(7, "existing", 0, 0);
            }
            GraphBuilder builder(&graph);
            {
                CoutSilencer quiet;
                builder.addNodesFromCoordinates({{1, 2}, {3, 4}});
            }
            const bool numbered = graph.hasNode(8) && graph.hasNode(9) && graph.getNode(9).getX() == 3 &&
                                  graph.getNode(8).getName() == "node_8";
            const bool rejected = throws<std::invalid_argument>([&] {
                builder.addNodesFromCoordinates({{5, 5}, {std::nan(""), 0}});
            });
            return numbered && rejected && graph.getNodeCount() == 3;
        });

        runTest("connectWithinRadius joins exactly the pairs within the radius and keeps existing edges", []() {
            std::mt19937 rng(20261110);
            Graph graph = randomGraph(rng, 40, 0);
            {
                CoutSilencer quiet;
                graph.addEdge(0, 1, 99.0);
            }
            GraphBuilder builder(&graph);
            {
                CoutSilencer quiet;
                builder.connectWithinRadius(18.0);
            }
            bool exact = graph.getEdgeWeight(0, 1) == 99.0;
            for (int a = 0; a < 40; ++a) {
                for (int b = a + 1; b < 40; ++b) {
                    if (a == 0 && b == 1) continue;
                    const bool within = distance(graph, a, b) <= 18.0;
                    exact = exact && graph.hasEdge(a, b) == within && graph.hasEdge(b, a) == within &&
                            (!within || graph.getEdgeWeight(a, b) == distance(graph, a, b));
                }
            }
            return exact && throws<std::invalid_argument>([&] { builder.connectWithinRadius(-1.0); });
        });

        runTest("connectNearestNeighbors joins every node to its k nearest", []() {
            std::mt19937 rng(20261111);
            Graph graph = randomGraph(rng, 30, 0);
            GraphBuilder builder(&graph);
            {
                CoutSilencer quiet;
                builder.connectNearestNeighbors(3);
            }
            bool all = true;
            for (int a = 0; a < 30; ++a) {
                std::vector<std::pair<double, int>> others;
                for (int b = 0; b < 30; ++b) {
                    if (b != a) others.emplace_back(distance(graph, a, b), b);
                }
                std::sort(others.begin(), others.end());
                for (int i = 0; i < 3; ++i) {
                    all = all && graph.hasEdge(a, others[i].second) && graph.hasEdge(others[i].second, a);
                }
            }
            return all && throws<std::invalid_argument>([&] { builder.connectNearestNeighbors(0); });
        });

        runTest("generateMazeLayout builds a spanning tree of the grid", []() {
            Graph graph;
            GraphBuilder builder(&graph);
            {
                CoutSilencer quiet;
                builder.generateMazeLayout(9, 7);
            }
            const auto pairs = undirectedPairs(graph);
            bool orthogonal = true;
            for (const auto& [a, b] : pairs) {
                orthogonal = orthogonal && ((b - a == 1 && a / 9 == b / 9) || b - a == 9);
            }
            Graph nonEmpty;
            {
                CoutSilencer quiet;
                nonEmpty.addNode(0, "n", 0, 0);
            }
            GraphBuilder onNonEmpty(&nonEmpty);
            return graph.getNodeCount() == 63 && pairs.size() == 62 && orthogonal && componentCount(graph) == 1 &&
                   throws<std::invalid_argument>([&] { onNonEmpty.generateMazeLayout(3, 3); });
        });

        runTest("moveNode moves a node and keeps its edges; refuses unknown, non-finite or taken positions", []() {
            Graph graph;
            {
                CoutSilencer quiet;
                graph.addNode(0, "a", 0, 0);
                graph.addNode(1, "b", 5, 0);
                graph.addEdge(0, 1, 5.0);
            }
            NodeManager nodes(&graph);
            const std::uint64_t before = graph.getRevision();
            const bool moved = nodes.moveNode(0, 1, 1);
            const bool kept = graph.getNode(0).getX() == 1 && graph.getNode(0).getY() == 1 &&
                              graph.getNode(0).getName() == "a" && graph.hasEdge(0, 1) && graph.getEdgeWeight(0, 1) == 5.0;
            return moved && kept && graph.getRevision() != before && !nodes.moveNode(9, 0, 0) &&
                   !nodes.moveNode(0, std::nan(""), 0) && !nodes.moveNode(0, 5, 0) && nodes.moveNode(0, 1, 1);
        });

        runTest("renameNode renames a node and refuses unknown ids", []() {
            Graph graph;
            {
                CoutSilencer quiet;
                graph.addNode(3, "old", 2, 4);
            }
            NodeManager nodes(&graph);
            return nodes.renameNode(3, "new") && graph.getNode(3).getName() == "new" && graph.getNode(3).getX() == 2 &&
                   !nodes.renameNode(4, "missing");
        });

        runTest("getMinimumSpanningTree satisfies the cycle property on 200 random graphs", []() {
            std::mt19937 rng(20261112);
            for (int trial = 0; trial < 200; ++trial) {
                Graph graph = randomGraph(rng, 3 + static_cast<int>(rng() % 10), static_cast<int>(rng() % 25));
                EdgeManager edges(&graph);
                const auto forest = edges.getMinimumSpanningTree();
                // Cheapest weight per undirected pair, as the contract defines it.
                std::map<std::pair<int, int>, double> weight;
                for (int id : graph.getAllNodeIds()) {
                    for (const Edge& e : graph.getEdgesFrom(id)) {
                        const auto key = std::make_pair(std::min(id, e.getToNode()), std::max(id, e.getToNode()));
                        weight[key] = weight.count(key) ? std::min(weight[key], e.getWeight()) : e.getWeight();
                    }
                }
                std::unordered_map<int, std::vector<std::pair<int, double>>> tree;
                for (const auto& [a, b] : forest) {
                    if (!weight.count({a, b})) return false;
                    tree[a].push_back({b, weight[{a, b}]});
                    tree[b].push_back({a, weight[{a, b}]});
                }
                if (static_cast<int>(forest.size()) != static_cast<int>(graph.getNodeCount()) - componentCount(graph)) {
                    return false;  // a spanning forest has n - c edges (fewer means a component is split)
                }
                // Cycle property: every other edge weighs at least the heaviest edge on the tree path it closes.
                for (const auto& [pair, w] : weight) {
                    std::function<bool(int, int, double, double&)> heaviest = [&](int at, int from, double sofar, double& out) {
                        if (at == pair.second) {
                            out = sofar;
                            return true;
                        }
                        for (const auto& [next, edgeWeight] : tree[at]) {
                            if (next != from && heaviest(next, at, std::max(sofar, edgeWeight), out)) return true;
                        }
                        return false;
                    };
                    double maxOnPath = 0.0;
                    if (!heaviest(pair.first, -1, 0.0, maxOnPath) || w < maxOnPath) {
                        return false;
                    }
                }
            }
            return true;
        });

        runTest("getBridgeEdges matches brute force on 300 random graphs", []() {
            std::mt19937 rng(20261113);
            for (int trial = 0; trial < 300; ++trial) {
                const Graph graph = randomGraph(rng, 2 + static_cast<int>(rng() % 11), static_cast<int>(rng() % 16));
                ConnectivityAnalyzer analyzer(&graph);
                const auto bridges = analyzer.getBridgeEdges();
                std::vector<std::pair<int, int>> expected;
                const int components = componentCount(graph);
                for (const auto& pair : undirectedPairs(graph)) {
                    Graph without = graph;
                    {
                        CoutSilencer quiet;
                        without.removeEdge(pair.first, pair.second);
                        without.removeEdge(pair.second, pair.first);
                    }
                    if (componentCount(without) > components) {
                        expected.push_back(pair);
                    }
                }
                if (bridges != expected) {
                    std::cout << "(trial " << trial << ") ";
                    return false;
                }
            }
            return true;
        });

        runTest("saveCostModel and loadCostModel round-trip every value exactly", []() {
            Graph graph;
            {
                CoutSilencer quiet;
                graph.addNode(0, "a", 0, 0);
                graph.addNode(1, "b", 1, 0);
                graph.addEdge(0, 1, 1.0);
            }
            CostCalculator saved(&graph);
            saved.setCostWeight(CostType::TIME, 1.0 / 3.0);
            saved.setNodeCost(1, 0.1);
            saved.setEdgeCost(0, 1, 2.0 / 7.0);
            const std::string file = "test_cost_model.txt";
            saved.saveCostModel(file);
            CostCalculator loaded(&graph);
            loaded.loadCostModel(file);
            std::remove(file.c_str());
            const std::vector<int> path{0, 1};
            return loaded.getSegmentCost(0, 1) == saved.getSegmentCost(0, 1) &&
                   loaded.getNodeTraversalCost(1) == saved.getNodeTraversalCost(1) &&
                   loaded.calculatePathCostWithType(path, CostType::TIME) ==
                       saved.calculatePathCostWithType(path, CostType::TIME) &&
                   loaded.calculatePathCost(path) == saved.calculatePathCost(path);
        });

        runTest("loadCostModel rejects malformed files and keeps the current model", []() {
            Graph graph;
            {
                CoutSilencer quiet;
                graph.addNode(0, "a", 0, 0);
                graph.addNode(1, "b", 1, 0);
                graph.addEdge(0, 1, 1.0);
            }
            CostCalculator model(&graph);
            model.setEdgeCost(0, 1, 5.0);
            const std::string file = "test_cost_model_bad.txt";
            const std::vector<std::string> bad{"not a model\n", "cost-model 1\nweight SPEED 1\n",
                                               "cost-model 1\nweight TIME -1\n", "cost-model 1\nedge 0 1\n",
                                               "cost-model 1\nnode 1 2 extra\n", ""};
            bool allRejected = true;
            for (const std::string& content : bad) {
                {
                    std::ofstream out(file, std::ios::binary);
                    out << content;
                }
                allRejected = allRejected && throws<std::runtime_error>([&] { model.loadCostModel(file); });
            }
            std::remove(file.c_str());
            return allRejected && model.getSegmentCost(0, 1) == 5.0 &&
                   throws<std::runtime_error>([&] { model.loadCostModel("no_such_cost_model.txt"); });
        });

        runTest("preserveCriticalWaypoints keeps a protected corner that smoothing would cut", []() {
            Graph graph;
            {
                CoutSilencer quiet;
                graph.addNode(0, "a", 0, 0);
                graph.addNode(1, "corner", 10, 0);
                graph.addNode(2, "c", 10, 10);
                graph.addEdge(0, 1, 10.0);
                graph.addEdge(1, 2, 10.0);
                graph.addEdge(0, 2, 14.2);
            }
            PathSmoother smoother(&graph);
            const std::vector<int> rough{0, 1, 2};
            const std::vector<int> cut = smoother.smoothPath(rough);
            smoother.preserveCriticalWaypoints({1});
            const std::vector<int> kept = smoother.smoothPath(rough);
            smoother.preserveCriticalWaypoints({});
            const std::vector<int> cutAgain = smoother.smoothPath(rough);
            return cut == std::vector<int>{0, 2} && kept == rough && cutAgain == std::vector<int>{0, 2};
        });

        std::cout << "\n=== Graph Operations API Test Results ===" << std::endl;
        std::cout << "Tests passed: " << testsPassed << "/" << testsRun << std::endl;
    }

    bool allPassed() const {
        return testsRun > 0 && testsPassed == testsRun;
    }
};

int main() {
    GraphOperationsApiTester tester;
    tester.runAllTests();
    return tester.allPassed() ? 0 : 1;
}

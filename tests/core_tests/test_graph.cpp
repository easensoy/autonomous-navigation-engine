#include "core/Graph.hpp"
#include <exception>
#include <functional>
#include <iostream>
#include <string>

// Edge bookkeeping in Graph. A two-way edge is one edge stored in both directions, so removing it
// removes both directions and it counts once. Two one-way edges between the same nodes are two
// separate edges.
class GraphTester {
private:
    int testsRun = 0;
    int testsPassed = 0;

    static Graph makeNodes(int count) {
        Graph graph;
        for (int i = 0; i < count; ++i) {
            graph.addNode(i, "Node" + std::to_string(i), i * 10.0, 0.0);
        }
        return graph;
    }

    void runTest(const std::string& testName, std::function<bool()> test) {
        testsRun++;
        std::cout << "[TEST] " << testName << ": ";

        try {
            if (test()) {
                testsPassed++;
                std::cout << "PASS" << std::endl;
            } else {
                std::cout << "FAIL" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cout << "EXCEPTION: " << e.what() << std::endl;
        }
    }

public:
    void runAllTests() {
        std::cout << "\n=== Graph Tests ===" << std::endl;

        runTest("Removing a two-way edge removes both directions", []() {
            Graph graph = makeNodes(2);
            graph.addEdge(0, 1, 10.0);  // two-way by default
            const bool removed = graph.removeEdge(0, 1);
            return removed && !graph.hasEdge(0, 1) && !graph.hasEdge(1, 0);
        });

        runTest("Removing one of two one-way edges keeps the other", []() {
            Graph graph = makeNodes(2);
            graph.addEdge(0, 1, 10.0, false);
            graph.addEdge(1, 0, 20.0, false);
            const bool removed = graph.removeEdge(0, 1);
            return removed && !graph.hasEdge(0, 1) && graph.hasEdge(1, 0) && graph.getEdgeWeight(1, 0) == 20.0;
        });

        runTest("Removing a missing edge returns false and changes nothing", []() {
            Graph graph = makeNodes(3);
            graph.addEdge(0, 1, 10.0);
            const bool removed = graph.removeEdge(1, 2);
            return !removed && graph.hasEdge(0, 1) && graph.hasEdge(1, 0);
        });

        runTest("Edge count counts one-way and two-way edges once each", []() {
            Graph graph = makeNodes(3);
            graph.addEdge(0, 1, 10.0);         // two-way
            graph.addEdge(1, 2, 10.0, false);  // one-way
            graph.addEdge(2, 0, 10.0, false);  // one-way
            return graph.getEdgeCount() == 3;
        });

        runTest("Edge count drops by one when a two-way edge is removed", []() {
            Graph graph = makeNodes(3);
            graph.addEdge(0, 1, 10.0);
            graph.addEdge(1, 2, 10.0);
            graph.removeEdge(0, 1);
            return graph.getEdgeCount() == 1 && !graph.hasEdge(1, 0);
        });

        runTest("Revision changes with every edit and not on read-only access", []() {
            Graph graph = makeNodes(2);
            const auto initial = graph.getRevision();
            graph.addEdge(0, 1, 10.0);
            const auto afterAdd = graph.getRevision();
            graph.removeEdge(0, 1);
            const auto afterRemove = graph.getRevision();
            const Graph& readOnly = graph;
            readOnly.getNode(0);
            readOnly.getEdgesFrom(0);
            const auto afterRead = graph.getRevision();
            graph.removeNode(1);
            const auto afterNodeRemoval = graph.getRevision();
            return afterAdd != initial && afterRemove != afterAdd && afterRead == afterRemove &&
                   afterNodeRemoval != afterRead;
        });

        printResults();
    }

    void printResults() {
        std::cout << "\n=== Graph Test Results ===" << std::endl;
        std::cout << "Tests passed: " << testsPassed << "/" << testsRun << std::endl;

        if (testsPassed == testsRun) {
            std::cout << "✓ All Graph tests passed!" << std::endl;
        } else {
            std::cout << "✗ Some Graph tests failed!" << std::endl;
        }
    }

    bool allPassed() const {
        return testsRun > 0 && testsPassed == testsRun;
    }
};

int main() {
    GraphTester tester;
    tester.runAllTests();
    return tester.allPassed() ? 0 : 1;
}

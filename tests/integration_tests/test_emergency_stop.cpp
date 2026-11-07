#include "autonomous_navigator/PathExecutor.hpp"
#include "core/Graph.hpp"
#include <chrono>
#include <cstdlib>
#include <exception>
#include <functional>
#include <future>
#include <iostream>
#include <string>
#include <thread>

// The emergency-stop state is shared by the whole process and the public API cannot clear it, so
// these checks run in their own program, starting from a process where no stop has happened yet.
class EmergencyStopTester {
private:
    int testsRun = 0;
    int testsPassed = 0;

    static void makeSegment(Graph& graph) {
        graph.addNode(0, "Start", 0, 0);
        graph.addNode(1, "Goal", 10, 0);
        graph.addEdge(0, 1, 10.0);
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
        std::cout << "\n=== Emergency Stop Tests ===" << std::endl;

        runTest("Error callback can restart execution during an emergency stop", []() {
            // Runs on a worker thread so that a deadlock shows up as a timeout, not a hung test.
            std::promise<int> outcome;  // error notifications received, or -1 if an exception escaped
            std::future<int> result = outcome.get_future();
            std::thread worker([&outcome]() {
                try {
                    Graph graph;
                    makeSegment(graph);
                    PathExecutor executor(&graph);
                    executor.loadPath({0, 1});
                    int notifications = 0;
                    executor.setErrorCallback([&executor, &notifications](const std::string&) {
                        ++notifications;
                        executor.startExecution();  // calls back into the executor while the stop is handled
                    });
                    executor.startExecution();
                    executor.emergencyStop();
                    outcome.set_value(notifications);
                } catch (const std::exception& e) {
                    std::cout << "(exception: " << e.what() << ") ";
                    outcome.set_value(-1);
                }
            });
            if (result.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
                std::cout << "FAIL (emergencyStop did not return within 10 s: deadlock)" << std::endl;
                std::_Exit(1);  // the stuck thread cannot be joined, so end the program here
            }
            worker.join();
            const int notifications = result.get();
            std::cout << "(" << notifications << " notification(s)) ";
            return notifications == 1;
        });

        runTest("Emergency stop while one is already active notifies once", []() {
            Graph graph;
            makeSegment(graph);
            PathExecutor executor(&graph);
            executor.loadPath({0, 1});
            int notifications = 0;
            executor.setErrorCallback([&notifications](const std::string&) { ++notifications; });
            executor.emergencyStop();  // the stop from the previous test is still active
            return notifications == 1;
        });

        printResults();
    }

    void printResults() {
        std::cout << "\n=== Emergency Stop Test Results ===" << std::endl;
        std::cout << "Tests passed: " << testsPassed << "/" << testsRun << std::endl;

        if (testsPassed == testsRun) {
            std::cout << "✓ All emergency stop tests passed!" << std::endl;
        } else {
            std::cout << "✗ Some emergency stop tests failed!" << std::endl;
        }
    }

    bool allPassed() const {
        return testsRun > 0 && testsPassed == testsRun;
    }
};

int main() {
    EmergencyStopTester tester;
    tester.runAllTests();
    return tester.allPassed() ? 0 : 1;
}

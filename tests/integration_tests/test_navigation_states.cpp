#include "autonomous_navigator/NavigationCore.hpp"
#include "core/Graph.hpp"
#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

// NavigationCore's state machine, in particular what a failed replan leaves behind (see
// NavigationCore.hpp): BLOCKED is distinguishable from IDLE, the invalidated route is never returned
// as the current path, no step is taken, and recovery goes through retryNavigation or a new start.

namespace {

// A corridor 0-1-2-3-4 (weight 1 per edge) with a costlier detour 1-5-3 (weight 1.5 per edge).
Graph makeCorridor() {
    Graph graph;
    for (int x = 0; x < 5; ++x) {
        graph.addNode(x, "c" + std::to_string(x), x, 0);
    }
    graph.addNode(5, "detour", 2, 1);
    for (int x = 0; x < 4; ++x) {
        graph.addEdge(x, x + 1, 1.0);
    }
    graph.addEdge(1, 5, 1.5);
    graph.addEdge(5, 3, 1.5);
    return graph;
}

// Starts 0 -> 4 and takes one step, so the navigator stands on node 1.
bool startAndStep(NavigationCore& navigator) {
    return navigator.startNavigation(0, 4) && navigator.getCurrentPath() == std::vector<int>{0, 1, 2, 3, 4} &&
           navigator.executeNavigationStep() && navigator.getCurrentPosition() == 1;
}

bool isBlockedAt(const NavigationCore& navigator, int position, const std::vector<int>& travelled,
                 const std::vector<int>& invalidated) {
    return navigator.getNavigationState() == NavigationState::BLOCKED &&
           navigator.getCurrentPosition() == position && navigator.getCurrentPath() == travelled &&
           navigator.getInvalidatedRoute() == invalidated && std::isinf(navigator.getRemainingDistance()) &&
           !navigator.isNavigationComplete() && navigator.getNavigationStatus().rfind("BLOCKED", 0) == 0;
}

}  // namespace

class NavigationStateTester {
private:
    int testsRun = 0;
    int testsPassed = 0;

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
        std::cout << "\n=== Navigation State Tests ===" << std::endl;

        runTest("A new navigator is IDLE with no route", []() {
            Graph graph = makeCorridor();
            NavigationCore navigator(&graph);
            return navigator.getNavigationState() == NavigationState::IDLE && navigator.getCurrentPath().empty() &&
                   navigator.getInvalidatedRoute().empty() && navigator.getCurrentPosition() == -1 &&
                   navigator.getGoal() == -1 && !navigator.executeNavigationStep() &&
                   navigator.getRemainingDistance() == 0.0;
        });

        runTest("Successful navigation ends COMPLETED and then refuses to step", []() {
            Graph graph = makeCorridor();
            NavigationCore navigator(&graph);
            if (!navigator.startNavigation(0, 4) || navigator.getNavigationState() != NavigationState::ACTIVE) {
                return false;
            }
            int steps = 0;
            while (navigator.executeNavigationStep()) {
                ++steps;
            }
            return steps == 4 && navigator.getNavigationState() == NavigationState::COMPLETED &&
                   navigator.isNavigationComplete() && navigator.getCurrentPosition() == 4 &&
                   navigator.getRemainingDistance() == 0.0 && !navigator.executeNavigationStep();
        });

        runTest("A blocked goal makes the navigator BLOCKED, keeping only the travelled part", []() {
            Graph graph = makeCorridor();
            NavigationCore navigator(&graph);
            if (!startAndStep(navigator)) {
                return false;
            }
            navigator.handleObstacles({4});
            return isBlockedAt(navigator, 1, {0, 1}, {1, 2, 3, 4}) && navigator.getGoal() == 4;
        });

        runTest("A goal cut off by obstacles makes the navigator BLOCKED", []() {
            Graph graph = makeCorridor();
            NavigationCore navigator(&graph);
            if (!startAndStep(navigator)) {
                return false;
            }
            navigator.handleObstacles({2, 5});  // both ways to node 3, and so to the goal, are blocked
            return isBlockedAt(navigator, 1, {0, 1}, {1, 2, 3, 4});
        });

        runTest("BLOCKED refuses to advance; pause, resume and emergency stop leave it BLOCKED", []() {
            Graph graph = makeCorridor();
            NavigationCore navigator(&graph);
            if (!startAndStep(navigator)) {
                return false;
            }
            navigator.handleObstacles({4});
            bool stepped = false;
            for (int i = 0; i < 3; ++i) {
                stepped = stepped || navigator.executeNavigationStep();
            }
            navigator.pauseNavigation();
            navigator.resumeNavigation();
            navigator.emergencyStop();
            return !stepped && isBlockedAt(navigator, 1, {0, 1}, {1, 2, 3, 4});
        });

        runTest("Repeated failures: more obstacles and failed retries change nothing", []() {
            Graph graph = makeCorridor();
            NavigationCore navigator(&graph);
            if (!startAndStep(navigator)) {
                return false;
            }
            navigator.handleObstacles({2, 5});
            const bool firstRetry = navigator.retryNavigation();
            navigator.handleObstacles({3});
            const bool secondRetry = navigator.retryNavigation();
            return !firstRetry && !secondRetry && isBlockedAt(navigator, 1, {0, 1}, {1, 2, 3, 4});
        });

        runTest("Recovery: clearing an obstacle and retrying resumes and completes the route", []() {
            Graph graph = makeCorridor();
            NavigationCore navigator(&graph);
            if (!startAndStep(navigator)) {
                return false;
            }
            navigator.handleObstacles({2, 5});
            navigator.clearObstacles({5});  // the detour reopens; node 2 stays blocked
            if (!navigator.retryNavigation() || navigator.getNavigationState() != NavigationState::ACTIVE ||
                navigator.getCurrentPath() != std::vector<int>{0, 1, 5, 3, 4} ||
                !navigator.getInvalidatedRoute().empty() || navigator.getRemainingDistance() != 4.0) {
                return false;
            }
            std::vector<int> visited;
            while (navigator.executeNavigationStep()) {
                visited.push_back(navigator.getCurrentPosition());
            }
            return visited == std::vector<int>{5, 3, 4} && navigator.isNavigationComplete();
        });

        runTest("Recovery: a new start replaces a blocked navigation", []() {
            Graph graph = makeCorridor();
            NavigationCore navigator(&graph);
            if (!startAndStep(navigator)) {
                return false;
            }
            navigator.handleObstacles({4});
            const bool started = navigator.startNavigation(navigator.getCurrentPosition(), 3);
            return started && navigator.getNavigationState() == NavigationState::ACTIVE &&
                   navigator.getCurrentPath() == std::vector<int>{1, 2, 3} && navigator.getInvalidatedRoute().empty() &&
                   navigator.getGoal() == 3;
        });

        runTest("A failed start leaves the current navigation unchanged", []() {
            Graph graph = makeCorridor();
            NavigationCore navigator(&graph);
            if (!startAndStep(navigator)) {
                return false;
            }
            navigator.handleObstacles({5});  // detour blocked; the corridor ahead is still clear
            const bool unknown = navigator.startNavigation(1, 99);
            navigator.clearAllObstacles();
            navigator.handleObstacles({2, 5});  // forces a replan, which fails: BLOCKED
            const bool blockedStart = navigator.startNavigation(1, 3);  // node 3 is unreachable from 1 now
            return !unknown && !blockedStart && isBlockedAt(navigator, 1, {0, 1}, {1, 2, 3, 4});
        });

        runTest("An obstacle reported while PAUSED can block the navigation", []() {
            Graph graph = makeCorridor();
            NavigationCore navigator(&graph);
            if (!startAndStep(navigator)) {
                return false;
            }
            navigator.pauseNavigation();
            const bool paused = navigator.getNavigationState() == NavigationState::PAUSED;
            navigator.handleObstacles({4});
            navigator.resumeNavigation();  // no effect in BLOCKED
            return paused && isBlockedAt(navigator, 1, {0, 1}, {1, 2, 3, 4});
        });

        runTest("stopNavigation returns a blocked navigator to IDLE", []() {
            Graph graph = makeCorridor();
            NavigationCore navigator(&graph);
            if (!startAndStep(navigator)) {
                return false;
            }
            navigator.handleObstacles({4});
            navigator.stopNavigation();
            return navigator.getNavigationState() == NavigationState::IDLE && navigator.getCurrentPath().empty() &&
                   navigator.getInvalidatedRoute().empty() && navigator.getCurrentPosition() == -1 &&
                   navigator.getGoal() == -1 && navigator.getNavigationStatus().rfind("IDLE", 0) == 0;
        });

        runTest("A replan that succeeds keeps the navigator ACTIVE", []() {
            Graph graph = makeCorridor();
            NavigationCore navigator(&graph);
            if (!startAndStep(navigator)) {
                return false;
            }
            navigator.handleObstacles({2});
            return navigator.getNavigationState() == NavigationState::ACTIVE &&
                   navigator.getCurrentPath() == std::vector<int>{0, 1, 5, 3, 4} &&
                   navigator.getInvalidatedRoute().empty() && navigator.getRemainingDistance() == 4.0;
        });

        std::cout << "\n=== Navigation State Test Results ===" << std::endl;
        std::cout << "Tests passed: " << testsPassed << "/" << testsRun << std::endl;
    }

    bool allPassed() const {
        return testsRun > 0 && testsPassed == testsRun;
    }
};

int main() {
    NavigationStateTester tester;
    tester.runAllTests();
    return tester.allPassed() ? 0 : 1;
}

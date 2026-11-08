#pragma once
#include "core/Graph.hpp"
#include "navigation_strategies/GlobalPathPlanner.hpp"
#include "navigation_strategies/DynamicReplanner.hpp"
#include <memory>
#include <string>
#include <vector>

enum class NavigationMode {
    GLOBAL_ONLY,
    LOCAL_ONLY,
    HYBRID,
    DYNAMIC
};

// Navigation state. Transitions:
//   IDLE -> ACTIVE             startNavigation / startNavigationWithWaypoints finds a route of two or
//                              more nodes (one node, start == goal, gives COMPLETED)
//   ACTIVE -> COMPLETED        executeNavigationStep reaches the goal
//   ACTIVE -> PAUSED           pauseNavigation or emergencyStop; PAUSED -> ACTIVE on resumeNavigation
//   ACTIVE, PAUSED -> BLOCKED  handleObstacles finds no route from the current node to the goal
//   BLOCKED -> ACTIVE          retryNavigation finds a route (for example after clearObstacles)
//   any -> IDLE                stopNavigation or updateEnvironment
// A successful startNavigation replaces the current navigation in any state; a failed one changes
// nothing.
enum class NavigationState {
    IDLE,       // no navigation: never started, stopped, or the environment was replaced
    ACTIVE,     // following a route; executeNavigationStep advances one node
    PAUSED,     // following a route, held by pauseNavigation or emergencyStop
    COMPLETED,  // at the goal
    BLOCKED     // a replan found no route to the goal; the route ahead is invalidated
};

const char* navigationStateName(NavigationState state);

// Plans a route on a graph and follows it one node per executeNavigationStep call. Routes are planned
// by A* through the DynamicReplanner, which also avoids every obstacle reported with handleObstacles.
//
// Route ownership. In ACTIVE, PAUSED and COMPLETED, getCurrentPath returns the whole route from the
// start to the goal, including the travelled part. When handleObstacles leaves no route, the navigator
// enters BLOCKED and the route ahead stops being executable: getCurrentPath then returns only the
// travelled part (start to current node), getInvalidatedRoute returns the abandoned remainder (current
// node to goal, as it was), executeNavigationStep returns false without moving, and
// getRemainingDistance returns infinity. Recovery is retryNavigation, after the obstacle map changes
// (clearObstacles), or a new startNavigation.
class NavigationCore {
private:
    const Graph* environment;
    std::unique_ptr<GlobalPathPlanner> globalPlanner;
    std::unique_ptr<DynamicReplanner> dynamicReplanner;

    NavigationMode currentMode;
    NavigationState state;
    std::vector<int> activePath;        // the route; in BLOCKED only its travelled part
    std::vector<int> invalidatedRoute;  // BLOCKED only: the route ahead that could not be repaired
    int currentPosition;
    int goalNode;
    size_t routeIndex;  // index of currentPosition in activePath; a route may pass a node twice

    std::vector<int> planRoute(int startId, int goalId);
    bool activatePath(std::vector<int> path);

public:
    explicit NavigationCore(const Graph* graph);
    ~NavigationCore() = default;

    // Plan and begin a navigation. Return false, leaving the current navigation unchanged, when a node
    // is unknown or no route avoids the reported obstacles.
    bool startNavigation(int startId, int goalId);
    bool startNavigationWithWaypoints(int startId, const std::vector<int>& waypoints);
    void stopNavigation();
    void pauseNavigation();
    void resumeNavigation();

    NavigationState getNavigationState() const;
    std::vector<int> getCurrentPath() const;
    std::vector<int> getInvalidatedRoute() const;
    int getCurrentPosition() const;  // -1 in IDLE
    int getGoal() const;             // -1 in IDLE
    bool isNavigationComplete() const;
    double getRemainingDistance() const;  // edge weights ahead; infinity in BLOCKED, 0 in IDLE

    void setNavigationMode(NavigationMode mode);
    void updateEnvironment(const Graph* newEnvironment);

    // Reports blocked nodes. They stay known to the replanner until cleared. In ACTIVE, PAUSED and
    // COMPLETED the route ahead is kept if clear and replanned from the current node otherwise; with no
    // route left the navigator enters BLOCKED. In IDLE and BLOCKED the nodes are only recorded.
    void handleObstacles(const std::vector<int>& obstacles);
    // Forget reported obstacles. Neither call replans; use retryNavigation in BLOCKED.
    void clearObstacles(const std::vector<int>& nodes);
    void clearAllObstacles();
    // In BLOCKED, replans from the current node to the goal around the obstacles still known. On
    // success the route becomes the travelled part plus the new route, the state ACTIVE, and the result
    // true. Returns false and changes nothing otherwise.
    bool retryNavigation();

    // Advances one node in ACTIVE; returns false and does not move in any other state.
    bool executeNavigationStep();
    std::string getNavigationStatus() const;
    void emergencyStop();  // ACTIVE -> PAUSED; no effect in other states

    const Graph* getCurrentEnvironment() const;
};

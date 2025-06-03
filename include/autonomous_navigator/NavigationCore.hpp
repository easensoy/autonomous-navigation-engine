#pragma once
#include "core/Graph.hpp"
#include "navigation_strategies/GlobalPathPlanner.hpp"
#include "navigation_strategies/LocalPathPlanner.hpp"
#include "navigation_strategies/DynamicReplanner.hpp"
#include <memory>
#include <string>

enum class NavigationMode {
    GLOBAL_ONLY,
    LOCAL_ONLY,
    HYBRID,
    DYNAMIC
};

class NavigationCore {
private:
    const Graph* environment;
    std::unique_ptr<GlobalPathPlanner> globalPlanner;
    std::unique_ptr<LocalPathPlanner> localPlanner;
    std::unique_ptr<DynamicReplanner> dynamicReplanner;
    
    NavigationMode currentMode;
    std::vector<int> activePath;
    int currentPosition;
    bool navigationActive;

public:
    explicit NavigationCore(const Graph* graph);
    ~NavigationCore() = default;
    
    bool startNavigation(int startId, int goalId);
    bool startNavigationWithWaypoints(int startId, const std::vector<int>& waypoints);
    void stopNavigation();
    void pauseNavigation();
    void resumeNavigation();
    
    std::vector<int> getCurrentPath() const;
    int getCurrentPosition() const;
    bool isNavigationComplete() const;
    double getRemainingDistance() const;
    
    void setNavigationMode(NavigationMode mode);
    void updateEnvironment(const Graph* newEnvironment);
    void handleObstacles(const std::vector<int>& obstacles);
    
    bool executeNavigationStep();
    std::string getNavigationStatus() const;
    void emergencyStop();
    
    // Add this method to fix the compilation error
    const Graph* getCurrentEnvironment() const;
};
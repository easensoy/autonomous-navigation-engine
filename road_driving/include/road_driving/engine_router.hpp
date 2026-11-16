#pragma once
// The only boundary between road_driving and the Autonomous Navigation Engine. Lane segments and
// connectors become engine nodes (same ids), legal moves become one-way edges weighted by the travel
// time of the element entered. Routing and closure replanning are the engine's: NavigationCore plans
// with A* through its DynamicReplanner and avoids every element reported closed.
//
// Engine behaviours this adapter relies on or works around (audit 04, section 6):
//   - a closure reported on the node the engine currently occupies is ignored (N2): the caller must
//     check the occupied element itself;
//   - graph construction prints one line per node and edge to std::cout (N4): every engine call runs
//     with std::cout redirected to a counting sink;
//   - node coordinates beyond +-10000 graph units are rejected (N1): LaneNetwork::validate checks it;
//   - Graph::isConnected is not valid on one-way graphs (N3): never called.
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace road {

struct LaneNetwork;

class EngineRouter {
public:
    explicit EngineRouter(const LaneNetwork& net, double unitsPerMetre = 1.0);
    ~EngineRouter();
    EngineRouter(const EngineRouter&) = delete;
    EngineRouter& operator=(const EngineRouter&) = delete;

    bool start(int from, int goal);  // NavigationCore::startNavigation; avoids reported closures
    bool step();                     // executeNavigationStep: the vehicle entered the next element
    void reportClosed(const std::vector<int>& elements);  // handleObstacles
    void clear(const std::vector<int>& elements);         // clearObstacles (no replan)
    bool retry();                                         // retryNavigation (BLOCKED only)

    std::vector<int> routeAhead() const;  // current element to goal; empty unless ACTIVE or COMPLETED
    int position() const;
    std::string state() const;  // IDLE, ACTIVE, PAUSED, COMPLETED, BLOCKED
    bool active() const;
    bool blocked() const;
    bool consistent() const;  // the tracked route index points at the engine's current node

    double engineMilliseconds() const;  // wall clock spent in engine calls; never part of a trace
    std::uint64_t silencedBytes() const;
    std::size_t nodeCount() const;
    std::size_t edgeCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace road

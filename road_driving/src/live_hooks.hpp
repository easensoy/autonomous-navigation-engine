#pragma once
// Hooks into the episode loop, for live operation and recorded sessions (src/live.cpp). runEpisode passes
// none, so a headless episode's computations, random draws, sensor timestamps and logs are unchanged.
//
// A hooked episode calls beforeTick at the start of every 10 ms world tick, before the sensors step, and
// afterTick after the stack's control step, before the vehicle moves. beforeTick may inject world faults or
// place and lift a closure; both take effect at that tick, so a recorded (tick, command) list replays exactly.
// Nothing a hook sees reaches the estimator or the stack except through those world changes.
#include "road_driving/episode.hpp"
#include "road_driving/localisation.hpp"
#include "stack.hpp"

#include <vector>

namespace road {

// The world's closure as the episode loop holds it: the scenario's X1, or one an operator places.
struct LiveClosure {
    bool on = false;        // a closure exists in this episode
    bool present = false;   // the barrier stands
    bool revealed = false;  // the oracle has notified the stack
    bool lifted = false;
    int lane = -1;
    double barrierLaneS = 0.0, worksLength = 0.0;
    std::vector<int> closed;  // lane segments overlapping the works
};

// A read-only view of tick k.
struct TickView {
    long tick = 0;
    double t = 0.0;
    double dt = 0.01;
    const Scenario* scenario = nullptr;
    const LaneNetwork* net = nullptr;
    const VehicleParams* vehicle = nullptr;
    const VehicleState* truth = nullptr;              // the true state at t
    const Command* command = nullptr;                 // afterTick: the command computed at t
    const StateEstimate* estimate = nullptr;          // afterTick, estimated mode: the estimate the stack drove on
    const Estimator* estimator = nullptr;             // estimated mode
    const SensorSimulator* sensors = nullptr;         // with a sensor condition
    const std::vector<Measurement>* arrived = nullptr;  // afterTick: measurements released this tick
    const SensorZones* zones = nullptr;
    const Stack* stack = nullptr;
    bool stackStarted = false;
    const std::vector<LogEvent>* worldEvents = nullptr;
    const LiveClosure* closure = nullptr;
};

// What an operator changes at the start of a tick: faults first, then the closure.
struct TickControl {
    LiveFaults faults;          // stays in force until changed
    bool addClosure = false;    // place a closure (X1), notified to the stack at once
    int closureLane = -1;
    double barrierLaneS = 0.0, worksLength = 15.0;
    bool liftClosure = false;
};

class EpisodeHooks {
public:
    virtual ~EpisodeHooks() = default;
    // Start of tick k: set `control`; return false to end the episode now (reset or quit).
    virtual bool beforeTick(const TickView& view, TickControl& control) = 0;
    // After tick k's control step.
    virtual void afterTick(const TickView& view) = 0;
};

// runEpisode with hooks. With hooks == nullptr it is runEpisode. An episode a hook ends early has the
// outcome "ABORTED" and no evaluation.
EpisodeResult runEpisodeHooked(const Scenario& sc, EpisodeHooks* hooks);

}  // namespace road

#pragma once
// Live operation: the simulation stepped tick by tick behind a small line protocol, for the 3D viewer
// (road_driving/viewer) and for recorded sessions. The protocol is documented in viewer/PROTOCOL.md.
//
// The live server runs the same episode loop as a headless run (runEpisode), with hooks at two fixed
// points of every 10 ms tick. It only reads the simulation, except for the operator's world commands
// (GNSS outage and offset, kerb-scanner blackout, a closure and its lift), which take effect at a recorded
// tick. A session file holds the configuration and those (tick, command) pairs, so the same episode
// replays headlessly, bit for bit on the same toolchain. Rendering, pacing and pausing change nothing.
#include "road_driving/episode.hpp"

#include <iosfwd>
#include <string>

namespace road {

struct LiveOptions {
    double speed = 1.0;         // simulated seconds per wall-clock second; 0: as fast as possible
    int observeEvery = 1;       // ticks between "tick" messages (measurements and events are always sent)
    bool startPaused = false;
    // Commands come from a script, not an operator: end of input does not quit, and the server exits
    // when the episode ends.
    bool script = false;
    std::string initialConfig;  // JSON object: the first live episode; empty: wait for "reset"
    std::string replayPath;     // a session file to replay first
    std::string sessionOut;     // write each finished episode's session here (empty: only on "save_session")
};

// Runs until "quit", the end of input (unless scripted), or, scripted, the end of the episode. Messages
// go to `out`, one JSON object per line; human-readable diagnostics to `diag`. Returns 0 on success.
int runLiveServer(std::istream& in, std::ostream& out, std::ostream& diag, const LiveOptions& opt);

// The scenario a live configuration names ({"scenario": "R7-A", "condition": ..., "estimator": ...}),
// built as `road_drive drive` builds it.
bool scenarioFromConfig(const std::string& configJson, Scenario& sc, std::string& error);

// Replays a session file's episode headlessly: its configuration, with its commands at their ticks.
struct SessionReplay {
    EpisodeResult result;
    std::string recordedHash, recordedToolchain, recordedOutcome;  // empty when the session has no result
    int commands = 0;
};
bool replaySession(const std::string& sessionJson, SessionReplay& out, std::string& error);

// The map as the viewer draws it: junctions, streets, lanes, the exact kerb lines and corner arcs.
std::string sceneJson(const LaneNetwork& net);

}  // namespace road

#pragma once
// Reader for "ane-scenario 1" files (ros2/ane_sim/scenarios/*.txt), for the closed-loop test. The
// Gazebo trials read the same files with ane_sim/scripts/scenario_runner.py.
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ane::test {

struct ScenarioEvent {
    std::string trigger;  // at_start, x_ge, x_le, y_ge, y_le, time_ge
    double trigger_value = 0.0;
    std::string action;   // spawn_obstacle, close_region, odom_dropout, drive_stall
    std::vector<std::string> args;
};

struct Acceptance {
    std::string metric;
    std::string op;
    std::string value;
};

struct Scenario {
    std::string name;
    std::string title;
    std::string start;
    std::string goal;
    int trials = 1;
    unsigned long long seed = 0;
    double jitter_x = 0.0;
    double jitter_y = 0.0;
    double jitter_yaw = 0.0;
    double timeout = 120.0;
    double observe = 3.0;
    std::vector<ScenarioEvent> events;
    std::vector<Acceptance> accept;

    std::string expected(const std::string& metric) const {
        for (const Acceptance& a : accept) {
            if (a.metric == metric && a.op == "==") {
                return a.value;
            }
        }
        return {};
    }
};

inline Scenario loadScenario(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot open scenario " + path);
    }
    Scenario s;
    std::string raw;
    int number = 0;
    bool header = false;
    while (std::getline(in, raw)) {
        ++number;
        const std::string where = path + ":" + std::to_string(number);
        const std::size_t hash = raw.find('#');
        std::istringstream line(hash == std::string::npos ? raw : raw.substr(0, hash));
        std::string key;
        if (!(line >> key)) {
            continue;
        }
        std::vector<std::string> a;
        for (std::string token; line >> token;) {
            a.push_back(token);
        }
        auto need = [&](std::size_t n) {
            if (a.size() < n) {
                throw std::runtime_error(where + ": expected " + std::to_string(n) + " values after '" + key + "'");
            }
        };
        if (!header) {
            if (key != "ane-scenario" || a.size() != 1 || a[0] != "1") {
                throw std::runtime_error(where + ": expected 'ane-scenario 1'");
            }
            header = true;
        } else if (key == "name") {
            need(1);
            s.name = a[0];
        } else if (key == "title") {
            for (const std::string& word : a) {
                s.title += (s.title.empty() ? "" : " ") + word;
            }
        } else if (key == "start") {
            need(1);
            s.start = a[0];
        } else if (key == "goal") {
            need(1);
            s.goal = a[0];
        } else if (key == "trials") {
            need(1);
            s.trials = std::stoi(a[0]);
        } else if (key == "seed") {
            need(1);
            s.seed = std::stoull(a[0]);
        } else if (key == "initial_jitter") {
            need(3);
            s.jitter_x = std::stod(a[0]);
            s.jitter_y = std::stod(a[1]);
            s.jitter_yaw = std::stod(a[2]);
        } else if (key == "timeout") {
            need(1);
            s.timeout = std::stod(a[0]);
        } else if (key == "observe") {
            need(1);
            s.observe = std::stod(a[0]);
        } else if (key == "event") {
            need(2);
            ScenarioEvent e;
            e.trigger = a[0];
            std::size_t next = 1;
            if (e.trigger != "at_start") {
                need(3);
                e.trigger_value = std::stod(a[1]);
                next = 2;
            }
            e.action = a[next];
            e.args.assign(a.begin() + static_cast<std::ptrdiff_t>(next) + 1, a.end());
            s.events.push_back(e);
        } else if (key == "accept") {
            need(3);
            s.accept.push_back({a[0], a[1], a[2]});
        } else {
            throw std::runtime_error(where + ": unknown keyword '" + key + "'");
        }
    }
    if (!header || s.name.empty() || s.start.empty() || s.goal.empty()) {
        throw std::runtime_error(path + ": a scenario needs the header, name, start and goal");
    }
    return s;
}

}  // namespace ane::test

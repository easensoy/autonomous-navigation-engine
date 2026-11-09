// Prints the route this repository's engine plans on a roadmap, so an analysis can check that a
// published route came from the engine. Usage:
//   ane_route_check <roadmap file> <start node> <goal node> [blocked node ...]
// Plans with NavigationCore exactly as the navigator does (blocked nodes reported first, then
// startNavigation) and prints one line: "route <ids...>" and "cost <sum of edge weights>", or
// "no-route".
#include "ane_core/roadmap.hpp"
#include "autonomous_navigator/NavigationCore.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "usage: ane_route_check <roadmap file> <start node> <goal node> [blocked node ...]" << std::endl;
        return 2;
    }
    try {
        const ane::Roadmap map = ane::Roadmap::load(argv[1]);
        const int start = std::stoi(argv[2]);
        const int goal = std::stoi(argv[3]);
        std::vector<int> blocked;
        for (int i = 4; i < argc; ++i) {
            blocked.push_back(std::stoi(argv[i]));
        }
        std::ostringstream discard;
        std::streambuf* saved = std::cout.rdbuf(discard.rdbuf());  // NavigationCore logs to stdout
        NavigationCore nav(&map.graph());
        nav.handleObstacles(blocked);
        const bool ok = nav.startNavigation(start, goal);
        const std::vector<int> route = nav.getCurrentPath();
        std::cout.rdbuf(saved);
        if (!ok) {
            std::cout << "no-route" << std::endl;
            return 0;
        }
        std::cout << "route";
        for (int id : route) {
            std::cout << ' ' << id;
        }
        std::cout << "\ncost " << std::setprecision(17) << map.routeCost(route) << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ane_route_check: " << error.what() << std::endl;
        return 1;
    }
}

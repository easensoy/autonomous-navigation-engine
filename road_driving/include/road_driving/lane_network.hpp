#pragma once
// Synthetic left-hand-traffic lane network. Lane segments and turning movements (connectors) are the
// graph nodes, so one-way streets, banned turns and the absence of U-turns are expressed by which
// nodes and directed edges exist. Geometry is in metres; nothing here comes from a real street.
#include <cstddef>
#include <string>
#include <vector>

namespace road {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMph = 0.44704;  // metres per second per mile per hour

double wrapAngle(double a);  // to (-pi, pi]

struct PathPoint {
    double x = 0.0, y = 0.0, yaw = 0.0, kappa = 0.0, s = 0.0;  // s: arc length from the element start
};

enum class ElementKind { LaneSegment, Connector };
enum class Turn { None, Ahead, Left, Right };
const char* turnName(Turn turn);

struct Element {
    int id = -1;
    ElementKind kind = ElementKind::LaneSegment;
    std::string name;
    int lane = -1;         // lane segments: the owning lane
    int segment = -1;      // lane segments: index within the lane
    double laneS0 = 0.0;   // lane segments: lane arc length at the segment start
    int junction = -1;     // connectors
    int fromLane = -1;     // connectors
    int toLane = -1;       // connectors
    Turn turn = Turn::None;
    double speedLimit = 0.0;  // m/s
    double length = 0.0;      // m
    std::vector<PathPoint> pts;
    std::vector<int> next;  // legal successors (directed edges)
    std::vector<int> prev;
    bool isConnector() const { return kind == ElementKind::Connector; }
};

struct Junction {
    std::string name;
    double x = 0.0, y = 0.0;
};

struct Lane {
    std::string name;  // "J00>J10": from junction J00 towards J10
    int from = -1;     // junction index, -1 for a dead end
    int to = -1;
    double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;  // lane centreline start and end
    double dirX = 0.0, dirY = 0.0;                  // unit direction of travel
    double centreX0 = 0.0, centreY0 = 0.0;          // street centreline point beside the lane start
    double length = 0.0;
    std::vector<int> segments;
};

struct NetworkSpec {
    std::string name = "R1";
    double laneWidth = 3.0;
    double speedLimit = 20.0 * kMph;
    double mouth = 12.5;       // junction centre to the end of each approach lane and the start of each exit lane
    double deadEndGap = 2.0;   // lane end before a dead-end kerb
    double maxSegment = 20.0;  // lanes are cut into segments no longer than this
    double leftRadius = 7.5;   // lane-centre arc radius of left turns (6 m kerb radius + half a lane)
    double rightRadius = 10.0; // right turns cross the opposing lane in left-hand traffic
    double clothoid = 5.0;     // length of each curvature transition
    double penaltyLeft = 1.0, penaltyRight = 4.0, penaltyAhead = 0.0;  // engine cost, s
    double tieBreak = 1e-3;    // s; removes exact cost ties on the symmetric grid
    double sampleStep = 0.1;   // m between stored centreline points
};

struct BannedTurn {
    std::string junction, fromLane, toLane;
};

struct LaneNetwork {
    NetworkSpec spec;
    std::vector<Junction> junctions;
    std::vector<Lane> lanes;
    std::vector<Element> elements;
    std::vector<BannedTurn> banned;

    int junction(const std::string& name) const;   // -1 if absent
    int lane(const std::string& name) const;       // -1 if absent
    int connector(int fromLane, int toLane) const; // -1 if absent
    int segmentAt(int lane, double laneS) const;   // the segment containing a lane arc length
    double cost(int element) const;                // engine cost of entering and traversing an element, s
    PathPoint lanePoint(int lane, double laneS) const;
    // Empty when every invariant holds. unitsPerMetre is the graph scale handed to the engine, whose
    // nodes reject coordinates beyond +-10000 units.
    std::vector<std::string> validate(double unitsPerMetre = 1.0) const;
};

// Network R1: a 3 by 3 grid at x = 0, 120, 240 m and y = 0, 80, 160 m. Two-way streets with one 3.0 m
// lane each way at 20 mph, except the one-way eastbound middle street J01-J11-J21; no right turn from
// the northbound approach at J11 into it; a 60 m two-way cul-de-sac east of J21 with no turning head.
LaneNetwork buildR1(const NetworkSpec& spec = NetworkSpec{});
// A right-hand-traffic copy (x mirrored). validate() must reject it.
LaneNetwork mirrored(const LaneNetwork& net);

// Polyline distance from a point to an element's centreline: arc length and signed lateral offset
// (positive to the left of the direction of travel).
double distanceToElement(const Element& e, double x, double y, double* s = nullptr, double* lateral = nullptr);

// Independent Dijkstra over the network's own successor lists. Blocked elements other than the start
// are impassable.
struct RefRoute {
    std::vector<int> nodes;
    double cost = 0.0;
    bool found = false;
};
RefRoute referenceRoute(const LaneNetwork& net, int start, int goal, const std::vector<int>& blocked = {});
double routeCost(const LaneNetwork& net, const std::vector<int>& route);  // cost after the first element
bool routeLegal(const LaneNetwork& net, const std::vector<int>& route);
// The element nearest to a pose, among those within 45 degrees of its heading; -1 if none.
int associate(const LaneNetwork& net, double x, double y, double yaw, double* distance = nullptr);
std::string routeText(const LaneNetwork& net, const std::vector<int>& route);

}  // namespace road

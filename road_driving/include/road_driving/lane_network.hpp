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
    double kerbRadius = 6.0;       // corner kerb radius of an ordinary junction
    double tightKerbRadius = 3.0;  // corner kerb radius of a tight junction (side-road entry class)
    double tightLeftRadius = 5.5;  // left-turn arc at a tight junction; uses carriageway beyond its own lane
    double tightSpeed = 2.5;       // m/s, limit on a tight junction's left turns
};

struct BannedTurn {
    std::string junction, fromLane, toLane;
};

// A street's carriageway centreline between two junctions, or from a junction to a dead end (b = -1).
struct Street {
    int a = -1, b = -1;
    double ax = 0.0, ay = 0.0, bx = 0.0, by = 0.0;
};

// The kerb, the edge of the carriageway, as straight lines and corner arcs (built with the network).
struct KerbLine {
    double ax = 0.0, ay = 0.0, bx = 0.0, by = 0.0;
    double nx = 0.0, ny = 0.0;  // unit normal pointing into the carriageway
};
struct KerbArc {
    double cx = 0.0, cy = 0.0, r = 0.0;  // the carriageway lies outside this circle
    double mid = 0.0, half = 0.0;        // the arc spans the directions mid +- half seen from the centre
};

struct LaneNetwork {
    NetworkSpec spec;
    std::vector<Junction> junctions;
    std::vector<Lane> lanes;
    std::vector<Element> elements;
    std::vector<BannedTurn> banned;
    std::vector<Street> streets;
    std::vector<double> kerbRadius;  // per junction
    std::vector<KerbLine> kerbLines;
    std::vector<KerbArc> kerbArcs;

    int junction(const std::string& name) const;   // -1 if absent
    int lane(const std::string& name) const;       // -1 if absent
    int connector(int fromLane, int toLane) const; // -1 if absent
    int segmentAt(int lane, double laneS) const;   // the segment containing a lane arc length
    double cost(int element) const;                // engine cost of entering and traversing an element, s
    PathPoint lanePoint(int lane, double laneS) const;
    // Signed Euclidean distance from a point to the kerb (positive on the carriageway): street
    // carriageways two lanes wide, junction boxes, and corner kerbs rounded with the junction's radius.
    double roadClearance(double x, double y) const;
    // The same distance, and the unit normal at the nearest kerb point, pointing into the carriageway.
    double kerbDistance(double x, double y, double& nx, double& ny) const;
    // Distance along a ray (unit direction) to the first kerb it meets, or -1 within maxRange.
    double rayToKerb(double x, double y, double dx, double dy, double maxRange) const;
    // A conservative version of roadClearance: the same sign everywhere, but near a rounded corner it
    // understates the clearance of points inside a street's rectangle. Used for the inside test, and kept
    // because results up to November 2026 were measured with it.
    double coarseClearance(double x, double y) const;
    void buildKerbs();  // kerbLines and kerbArcs from streets, junctions and kerb radii
    // Empty when every invariant holds. unitsPerMetre is the graph scale handed to the engine, whose
    // nodes reject coordinates beyond +-10000 units.
    std::vector<std::string> validate(double unitsPerMetre = 1.0) const;
};

// The smallest roadClearance over a rectangular footprint (rear-axle pose; rear overhang behind it,
// front offset ahead of it, full width), sampled along its edges. Negative: part of it is off the road.
double footprintClearance(const LaneNetwork& net, double x, double y, double yaw, double rearOverhang, double frontOffset,
                          double width);

// A junction grid: junction Jcr at (xs[c], ys[r]); every street between neighbours is two-way unless
// listed one-way; banned turns are never built; dead ends add a cul-de-sac without a turning head;
// tight junctions get the small kerb radius and slow, wide left turns.
struct GridSpec {
    struct Directed {
        std::string from, to;
    };
    struct DeadEnd {
        std::string junction, name;
        double dx = 0.0, dy = 0.0;
    };
    std::string name;
    std::vector<double> xs, ys;
    std::vector<Directed> oneWay;
    std::vector<BannedTurn> banned;
    std::vector<DeadEnd> deadEnds;
    std::vector<std::string> tight;
    // Junction design overrides for this grid (<= 0: keep NetworkSpec's value).
    double mouth = -1.0, rightRadius = -1.0, tightLeftRadius = -1.0, tightKerbRadius = -1.0;
};
GridSpec gridR1();
GridSpec gridR2();
GridSpec gridR3();
GridSpec gridR4();
GridSpec gridR5();
GridSpec gridR6();
GridSpec gridR7();
LaneNetwork buildGrid(const GridSpec& grid, const NetworkSpec& spec = NetworkSpec{});

// Network R1 (development): a 3 by 3 grid at x = 0, 120, 240 m and y = 0, 80, 160 m. Two-way streets
// with one 3.0 m lane each way at 20 mph, except the one-way eastbound middle street J01-J11-J21; no
// right turn from the northbound approach at J11 into it; a 60 m cul-de-sac east of J21 with no
// turning head.
LaneNetwork buildR1(const NetworkSpec& spec = NetworkSpec{});
// Network R2 (held out): a 4 by 3 grid at x = 0, 90, 240, 350 m and y = 0, 70, 170 m; one-way streets
// J10->J11 (northbound) and J32->J22 (westbound); no right turn J11>J21 -> J21>J20 or J11>J12 ->
// J12>J22; tight junctions J11 and J31 (4 m kerbs, 5.5 m left-turn arcs at 2.5 m/s); 16 m junction mouths
// and 14 m right-turn arcs.
LaneNetwork buildR2(const NetworkSpec& spec = NetworkSpec{});
// Network R3 (held out for the second localisation evaluation): a 4 by 4 grid at x = 0, 110, 200, 330 m
// and y = 0, 85, 190, 270 m; one-way streets J12->J22 (eastbound) and J31->J30 (southbound); no right
// turn J11>J21 -> J21>J20 or J02>J12 -> J12>J11; tight junctions J21 and J13; a 50 m cul-de-sac east of
// J33. Junction design as R2's (16 m mouths, 14 m right-turn arcs, 4 m tight kerbs).
LaneNetwork buildR3(const NetworkSpec& spec = NetworkSpec{});
// Network R4 (held out for the evaluation of the second versions): a 5 by 3 grid at x = 0, 80, 170, 260,
// 380 m and y = 0, 100, 180 m; one-way streets J21->J31 (eastbound) and J02->J01 (southbound); no left
// turn J01>J11 -> J11>J12 and no right turn J22>J32 -> J32>J31; tight junctions J21 and J40; a 40 m
// cul-de-sac north of J42. Junction design as R2's.
LaneNetwork buildR4(const NetworkSpec& spec = NetworkSpec{});
// Network R5 (held out for the evaluation of the stack's re-join after pose corrections): a 3 by 5 grid at
// x = 0, 105, 230 m and y = 0, 75, 160, 235, 330 m; one-way streets J11->J12 (northbound) and J23->J13
// (westbound); no left turn J02>J12 -> J12>J13 and no right turn J13>J03 -> J03>J04; tight junctions J11 and
// J23; a 45 m cul-de-sac north of J24. Junction design as R2's.
LaneNetwork buildR5(const NetworkSpec& spec = NetworkSpec{});
// Network R6 (held out for the evaluation of the verified re-join): a 4 by 4 grid at x = 0, 85, 205, 300 m
// and y = 0, 110, 190, 300 m; one-way streets J10->J11 (northbound) and J32->J22 (westbound); no left turn
// J11>J21 -> J21>J22 and no right turn J22>J12 -> J12>J13; tight junctions J22 and J01; a 45 m cul-de-sac
// east of J30. Junction design as R2's.
LaneNetwork buildR6(const NetworkSpec& spec = NetworkSpec{});
// Network R7 (held out for the evaluation of the unique re-join): a 5 by 4 grid at x = 0, 90, 185, 270, 365 m
// and y = 0, 100, 195, 290 m; one-way streets J11->J21 (eastbound) and J33->J32 (southbound); no right turn
// J12>J22 -> J22>J21 and no left turn J30>J31 -> J31>J21; tight junctions J23 and J41; a 40 m cul-de-sac
// east of J42. Junction design as R2's.
LaneNetwork buildR7(const NetworkSpec& spec = NetworkSpec{});
// "R1" to "R7"; throws for any other name.
LaneNetwork buildNetwork(const std::string& name, const NetworkSpec& spec = NetworkSpec{});
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

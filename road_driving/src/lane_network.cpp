#include "road_driving/lane_network.hpp"

#include "road_driving/district.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace road {

double wrapAngle(double a) {
    a = std::fmod(a + kPi, 2.0 * kPi);
    if (a <= 0.0) {
        a += 2.0 * kPi;
    }
    return a - kPi;
}

const char* turnName(Turn turn) {
    switch (turn) {
        case Turn::Ahead: return "ahead";
        case Turn::Left: return "left";
        case Turn::Right: return "right";
        default: return "none";
    }
}

namespace {

struct Piece {
    double length, k0, k1;  // linear curvature from k0 to k1 over the length
};

// Samples consecutive linear-curvature pieces from a start pose, at most `step` apart and always at
// piece boundaries. Heading is exact; position is integrated with 16 midpoint substeps per sample.
std::vector<PathPoint> samplePieces(double x, double y, double yaw, const std::vector<Piece>& pieces, double step) {
    std::vector<PathPoint> out;
    out.push_back({x, y, wrapAngle(yaw), pieces.empty() ? 0.0 : pieces.front().k0, 0.0});
    double s = 0.0;
    const int sub = 16;
    for (const Piece& p : pieces) {
        if (p.length <= 1e-12) {
            continue;
        }
        const int n = std::max(1, static_cast<int>(std::ceil(p.length / step - 1e-9)));
        const double h = p.length / n;
        const double yaw0 = yaw;
        const double dk = (p.k1 - p.k0) / p.length;
        auto heading = [&](double u) { return yaw0 + p.k0 * u + 0.5 * dk * u * u; };
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < sub; ++j) {
                const double ua = (i + static_cast<double>(j) / sub) * h;
                const double ub = (i + static_cast<double>(j + 1) / sub) * h;
                const double th = heading(0.5 * (ua + ub));
                x += (ub - ua) * std::cos(th);
                y += (ub - ua) * std::sin(th);
            }
            const double u = (i + 1) * h;
            out.push_back({x, y, wrapAngle(heading(u)), p.k0 + dk * u, s + u});
        }
        yaw = heading(p.length);
        s += p.length;
    }
    return out;
}

std::vector<PathPoint> straight(double x0, double y0, double x1, double y1, double step) {
    const double len = std::hypot(x1 - x0, y1 - y0);
    const double yaw = std::atan2(y1 - y0, x1 - x0);
    const int n = std::max(1, static_cast<int>(std::ceil(len / step - 1e-9)));
    std::vector<PathPoint> out;
    out.reserve(static_cast<size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        out.push_back({x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, yaw, 0.0, len * t});
    }
    out.back().x = x1;  // exact shared endpoints with the neighbouring element
    out.back().y = y1;
    return out;
}

struct EndPoint {
    std::string name;
    double x, y;
    int junction;  // -1 for a dead end
};

char armLetter(double dx, double dy) {
    if (std::fabs(dx) >= std::fabs(dy)) {
        return dx > 0.0 ? 'E' : 'W';
    }
    return dy > 0.0 ? 'N' : 'S';
}

}  // namespace

int LaneNetwork::junction(const std::string& name) const {
    for (size_t i = 0; i < junctions.size(); ++i) {
        if (junctions[i].name == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

int LaneNetwork::lane(const std::string& name) const {
    for (size_t i = 0; i < lanes.size(); ++i) {
        if (lanes[i].name == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

int LaneNetwork::connector(int fromLane, int toLane) const {
    for (const Element& e : elements) {
        if (e.isConnector() && e.fromLane == fromLane && e.toLane == toLane) {
            return e.id;
        }
    }
    return -1;
}

int LaneNetwork::segmentAt(int laneIndex, double laneS) const {
    const Lane& l = lanes.at(static_cast<size_t>(laneIndex));
    for (int id : l.segments) {
        const Element& e = elements[static_cast<size_t>(id)];
        if (laneS < e.laneS0 + e.length) {
            return id;
        }
    }
    return l.segments.back();
}

double LaneNetwork::cost(int element) const {
    const Element& e = elements.at(static_cast<size_t>(element));
    double penalty = 0.0;
    if (e.turn == Turn::Left) {
        penalty = spec.penaltyLeft;
    } else if (e.turn == Turn::Right) {
        penalty = spec.penaltyRight;
    } else if (e.turn == Turn::Ahead) {
        penalty = spec.penaltyAhead;
    }
    const double frac = static_cast<double>((e.id * 37 + 11) % 101) / 101.0;
    return e.length / e.speedLimit + penalty + spec.tieBreak * frac;
}

PathPoint LaneNetwork::lanePoint(int laneIndex, double laneS) const {
    const Lane& l = lanes.at(static_cast<size_t>(laneIndex));
    const double s = std::min(std::max(laneS, 0.0), l.length);
    return {l.x0 + l.dirX * s, l.y0 + l.dirY * s, std::atan2(l.dirY, l.dirX), 0.0, s};
}

GridSpec gridR1() {
    GridSpec g;
    g.name = "R1";
    g.xs = {0.0, 120.0, 240.0};
    g.ys = {0.0, 80.0, 160.0};
    g.oneWay = {{"J01", "J11"}, {"J11", "J21"}};
    g.banned = {{"J11", "J10>J11", "J11>J21"}};
    g.deadEnds = {{"J21", "D21", 60.0, 0.0}};
    return g;
}

GridSpec gridR2() {
    GridSpec g;
    g.name = "R2";
    g.xs = {0.0, 90.0, 240.0, 350.0};
    g.ys = {0.0, 70.0, 170.0};
    g.oneWay = {{"J10", "J11"}, {"J32", "J22"}};
    g.banned = {{"J21", "J11>J21", "J21>J20"}, {"J12", "J11>J12", "J12>J22"}};
    g.tight = {"J11", "J31"};
    // Junction design checked by footprint clearance (road_drive map R2): with 16 m mouths, 14 m right-turn
    // arcs keep the swept footprint (with mirrors) on the carriageway; tight junctions need 4 m kerbs for
    // their 5.5 m left-turn arcs (3 m kerbs put the footprint over the kerb whatever the arc).
    g.mouth = 16.0;
    g.rightRadius = 14.0;
    g.tightKerbRadius = 4.0;
    return g;
}

GridSpec gridR3() {
    GridSpec g;
    g.name = "R3";
    g.xs = {0.0, 110.0, 200.0, 330.0};
    g.ys = {0.0, 85.0, 190.0, 270.0};
    g.oneWay = {{"J12", "J22"}, {"J31", "J30"}};
    g.banned = {{"J21", "J11>J21", "J21>J20"}, {"J12", "J02>J12", "J12>J11"}};
    g.deadEnds = {{"J33", "D33", 50.0, 0.0}};
    g.tight = {"J21", "J13"};
    // R2's junction design, which the exact footprint check (road_drive map R3) passes for every movement.
    g.mouth = 16.0;
    g.rightRadius = 14.0;
    g.tightKerbRadius = 4.0;
    return g;
}

LaneNetwork buildR1(const NetworkSpec& spec) { return buildGrid(gridR1(), spec); }

LaneNetwork buildR2(const NetworkSpec& spec) { return buildGrid(gridR2(), spec); }

LaneNetwork buildR3(const NetworkSpec& spec) { return buildGrid(gridR3(), spec); }

GridSpec gridR4() {
    GridSpec g;
    g.name = "R4";
    g.xs = {0.0, 80.0, 170.0, 260.0, 380.0};
    g.ys = {0.0, 100.0, 180.0};
    g.oneWay = {{"J21", "J31"}, {"J02", "J01"}};
    g.banned = {{"J11", "J01>J11", "J11>J12"}, {"J32", "J22>J32", "J32>J31"}};
    g.deadEnds = {{"J42", "D42", 0.0, 40.0}};
    g.tight = {"J21", "J40"};
    g.mouth = 16.0;
    g.rightRadius = 14.0;
    g.tightKerbRadius = 4.0;
    return g;
}

LaneNetwork buildR4(const NetworkSpec& spec) { return buildGrid(gridR4(), spec); }

GridSpec gridR5() {
    GridSpec g;
    g.name = "R5";
    g.xs = {0.0, 105.0, 230.0};
    g.ys = {0.0, 75.0, 160.0, 235.0, 330.0};
    g.oneWay = {{"J11", "J12"}, {"J23", "J13"}};
    g.banned = {{"J12", "J02>J12", "J12>J13"}, {"J03", "J13>J03", "J03>J04"}};
    g.deadEnds = {{"J24", "D24", 0.0, 45.0}};
    g.tight = {"J11", "J23"};
    g.mouth = 16.0;
    g.rightRadius = 14.0;
    g.tightKerbRadius = 4.0;
    return g;
}

LaneNetwork buildR5(const NetworkSpec& spec) { return buildGrid(gridR5(), spec); }

GridSpec gridR6() {
    GridSpec g;
    g.name = "R6";
    g.xs = {0.0, 85.0, 205.0, 300.0};
    g.ys = {0.0, 110.0, 190.0, 300.0};
    g.oneWay = {{"J10", "J11"}, {"J32", "J22"}};
    g.banned = {{"J21", "J11>J21", "J21>J22"}, {"J12", "J22>J12", "J12>J13"}};
    g.deadEnds = {{"J30", "D30", 45.0, 0.0}};
    g.tight = {"J22", "J01"};
    g.mouth = 16.0;
    g.rightRadius = 14.0;
    g.tightKerbRadius = 4.0;
    return g;
}

LaneNetwork buildR6(const NetworkSpec& spec) { return buildGrid(gridR6(), spec); }

GridSpec gridR7() {
    GridSpec g;
    g.name = "R7";
    g.xs = {0.0, 90.0, 185.0, 270.0, 365.0};
    g.ys = {0.0, 100.0, 195.0, 290.0};
    g.oneWay = {{"J11", "J21"}, {"J33", "J32"}};
    g.banned = {{"J22", "J12>J22", "J22>J21"}, {"J31", "J30>J31", "J31>J21"}};
    g.deadEnds = {{"J42", "D42", 40.0, 0.0}};
    g.tight = {"J23", "J41"};
    g.mouth = 16.0;
    g.rightRadius = 14.0;
    g.tightKerbRadius = 4.0;
    return g;
}

LaneNetwork buildR7(const NetworkSpec& spec) { return buildGrid(gridR7(), spec); }

LaneNetwork buildNetwork(const std::string& name, const NetworkSpec& spec) {
    if (name == "R4") {
        return buildR4(spec);
    }
    if (name == "R5") {
        return buildR5(spec);
    }
    if (name == "R6") {
        return buildR6(spec);
    }
    if (name == "R7") {
        return buildR7(spec);
    }
    if (name == "R1") {
        return buildR1(spec);
    }
    if (name == "R2") {
        return buildR2(spec);
    }
    if (name == "R3") {
        return buildR3(spec);
    }
    DistrictSpec district;
    if (findDistrict(name, district)) {
        return buildDistrict(district, spec);
    }
    throw std::invalid_argument("unknown network " + name);
}

LaneNetwork buildGrid(const GridSpec& grid, const NetworkSpec& baseSpec) {
    if (baseSpec.rightHand) {
        throw std::invalid_argument("grid " + grid.name + ": the grids are built for left-hand traffic only");
    }
    NetworkSpec spec = baseSpec;
    if (grid.mouth > 0.0) spec.mouth = grid.mouth;
    if (grid.rightRadius > 0.0) spec.rightRadius = grid.rightRadius;
    if (grid.tightLeftRadius > 0.0) spec.tightLeftRadius = grid.tightLeftRadius;
    if (grid.tightKerbRadius > 0.0) spec.tightKerbRadius = grid.tightKerbRadius;
    LaneNetwork net;
    net.spec = spec;
    net.spec.name = grid.name;
    const int nx = static_cast<int>(grid.xs.size()), ny = static_cast<int>(grid.ys.size());
    for (int r = 0; r < ny; ++r) {
        for (int c = 0; c < nx; ++c) {
            net.junctions.push_back({"J" + std::to_string(c) + std::to_string(r), grid.xs[static_cast<size_t>(c)],
                                     grid.ys[static_cast<size_t>(r)]});
        }
    }
    auto isTight = [&](const std::string& name) { return std::find(grid.tight.begin(), grid.tight.end(), name) != grid.tight.end(); };
    for (const Junction& jn : net.junctions) {
        net.kerbRadius.push_back(isTight(jn.name) ? spec.tightKerbRadius : spec.kerbRadius);
    }
    auto J = [&](int c, int r) {
        const int j = r * nx + c;
        return EndPoint{net.junctions[static_cast<size_t>(j)].name, grid.xs[static_cast<size_t>(c)], grid.ys[static_cast<size_t>(r)], j};
    };

    struct GridStreet {
        EndPoint a, b;
        bool ab, ba;  // lanes a -> b and b -> a exist
    };
    std::vector<GridStreet> streets;
    auto addStreet = [&](const EndPoint& a, const EndPoint& b) {
        GridStreet st{a, b, true, true};
        for (const GridSpec::Directed& ow : grid.oneWay) {
            if (ow.from == a.name && ow.to == b.name) {
                st.ba = false;
            }
            if (ow.from == b.name && ow.to == a.name) {
                st.ab = false;
            }
        }
        streets.push_back(st);
        net.streets.push_back({a.junction, b.junction, a.x, a.y, b.x, b.y, std::string(), 0.0, 0.0, 0.0, 0.0});  // a grid street: no district fields
    };
    for (int r = 0; r < ny; ++r) {
        for (int c = 0; c + 1 < nx; ++c) {
            addStreet(J(c, r), J(c + 1, r));
        }
    }
    for (int c = 0; c < nx; ++c) {
        for (int r = 0; r + 1 < ny; ++r) {
            addStreet(J(c, r), J(c, r + 1));
        }
    }
    for (const GridSpec::DeadEnd& de : grid.deadEnds) {
        const int j = net.junction(de.junction);
        const Junction& jn = net.junctions.at(static_cast<size_t>(j));
        addStreet(EndPoint{jn.name, jn.x, jn.y, j}, EndPoint{de.name, jn.x + de.dx, jn.y + de.dy, -1});  // no turning head
    }

    const double half = 0.5 * spec.laneWidth;
    auto addLane = [&](const EndPoint& p, const EndPoint& q) {
        Lane l;
        l.name = p.name + ">" + q.name;
        l.from = p.junction;
        l.to = q.junction;
        const double len = std::hypot(q.x - p.x, q.y - p.y);
        l.dirX = (q.x - p.x) / len;
        l.dirY = (q.y - p.y) / len;
        const double nx = -l.dirY, ny = l.dirX;  // left of the direction of travel: left-hand traffic
        const double inset0 = p.junction >= 0 ? spec.mouth : spec.deadEndGap;
        const double inset1 = q.junction >= 0 ? spec.mouth : spec.deadEndGap;
        l.centreX0 = p.x + l.dirX * inset0;
        l.centreY0 = p.y + l.dirY * inset0;
        l.x0 = l.centreX0 + nx * half;
        l.y0 = l.centreY0 + ny * half;
        l.x1 = q.x - l.dirX * inset1 + nx * half;
        l.y1 = q.y - l.dirY * inset1 + ny * half;
        l.length = len - inset0 - inset1;
        net.lanes.push_back(l);
    };
    for (const GridStreet& st : streets) {
        if (st.ab) {
            addLane(st.a, st.b);
        }
        if (st.ba) {
            addLane(st.b, st.a);
        }
    }

    // Lane segments.
    for (size_t li = 0; li < net.lanes.size(); ++li) {
        Lane& l = net.lanes[li];
        const int n = std::max(1, static_cast<int>(std::ceil(l.length / spec.maxSegment - 1e-9)));
        std::vector<double> ex(static_cast<size_t>(n) + 1), ey(static_cast<size_t>(n) + 1);
        for (int k = 0; k <= n; ++k) {
            const double s = l.length * k / n;
            ex[static_cast<size_t>(k)] = l.x0 + l.dirX * s;
            ey[static_cast<size_t>(k)] = l.y0 + l.dirY * s;
        }
        ex.back() = l.x1;
        ey.back() = l.y1;
        for (int k = 0; k < n; ++k) {
            Element e;
            e.id = static_cast<int>(net.elements.size());
            e.kind = ElementKind::LaneSegment;
            e.name = l.name + "/" + std::to_string(k);
            e.lane = static_cast<int>(li);
            e.segment = k;
            e.laneS0 = l.length * k / n;
            e.speedLimit = spec.speedLimit;
            const size_t a = static_cast<size_t>(k), b = a + 1;
            e.pts = straight(ex[a], ey[a], ex[b], ey[b], spec.sampleStep);
            e.length = e.pts.back().s;
            l.segments.push_back(e.id);
            net.elements.push_back(e);
        }
    }

    // Turning movements. U-turns do not exist; banned turns are listed and never built.
    net.banned = grid.banned;
    for (size_t j = 0; j < net.junctions.size(); ++j) {
        const Junction& jn = net.junctions[j];
        for (size_t in = 0; in < net.lanes.size(); ++in) {
            if (net.lanes[in].to != static_cast<int>(j)) {
                continue;
            }
            for (size_t out = 0; out < net.lanes.size(); ++out) {
                if (net.lanes[out].from != static_cast<int>(j)) {
                    continue;
                }
                const Lane& li = net.lanes[in];
                const Lane& lo = net.lanes[out];
                const double yin = std::atan2(li.dirY, li.dirX);
                const double yout = std::atan2(lo.dirY, lo.dirX);
                const double delta = wrapAngle(yout - yin);
                if (std::fabs(delta) > 0.75 * kPi) {
                    continue;  // a U-turn: not a movement in this network
                }
                bool isBanned = false;
                for (const BannedTurn& bt : net.banned) {
                    isBanned = isBanned || (bt.junction == jn.name && bt.fromLane == li.name && bt.toLane == lo.name);
                }
                if (isBanned) {
                    continue;
                }
                Element e;
                e.id = static_cast<int>(net.elements.size());
                e.kind = ElementKind::Connector;
                e.junction = static_cast<int>(j);
                e.fromLane = static_cast<int>(in);
                e.toLane = static_cast<int>(out);
                e.speedLimit = spec.speedLimit;
                const char armIn = armLetter(-li.dirX, -li.dirY);
                const char armOut = armLetter(lo.dirX, lo.dirY);
                e.name = jn.name + ":" + armIn + ">" + armOut;
                if (std::fabs(delta) < 0.1) {
                    e.turn = Turn::Ahead;
                    e.pts = straight(li.x1, li.y1, lo.x0, lo.y0, spec.sampleStep);
                } else {
                    e.turn = delta > 0.0 ? Turn::Left : Turn::Right;
                    const bool tightLeft = e.turn == Turn::Left && isTight(jn.name);
                    const double radius = tightLeft ? spec.tightLeftRadius : (e.turn == Turn::Left ? spec.leftRadius : spec.rightRadius);
                    if (tightLeft) {
                        e.speedLimit = spec.tightSpeed;
                    }
                    const double sign = delta > 0.0 ? 1.0 : -1.0;
                    const double k = sign / radius;
                    const double ls = spec.clothoid;
                    const double la = std::fabs(delta) * radius - ls;  // arc length so the total turn is delta
                    const std::vector<Piece> curve = {{ls, 0.0, k}, {la, k, k}, {ls, k, 0.0}};
                    const std::vector<PathPoint> local = samplePieces(0.0, 0.0, 0.0, curve, spec.sampleStep);
                    const double ex = local.back().x, ey = local.back().y;
                    const double rx = lo.x0 - li.x1 - (std::cos(yin) * ex - std::sin(yin) * ey);
                    const double ry = lo.y0 - li.y1 - (std::sin(yin) * ex + std::cos(yin) * ey);
                    const double runIn = rx * std::cos(yin) + ry * std::sin(yin);  // valid: perpendicular arms
                    const double runOut = rx * std::cos(yout) + ry * std::sin(yout);
                    const std::vector<Piece> pieces = {{std::max(runIn, 0.0), 0.0, 0.0}, {ls, 0.0, k}, {la, k, k}, {ls, k, 0.0},
                                                       {std::max(runOut, 0.0), 0.0, 0.0}};
                    e.pts = samplePieces(li.x1, li.y1, yin, pieces, spec.sampleStep);
                    if (runIn < -1e-9 || runOut < -1e-9 || std::hypot(e.pts.back().x - lo.x0, e.pts.back().y - lo.y0) > 1e-6) {
                        e.name += "(geometry-misfit)";  // reported by validate()
                    }
                    e.pts.back().x = lo.x0;
                    e.pts.back().y = lo.y0;
                }
                e.length = e.pts.back().s;
                net.elements.push_back(e);
            }
        }
    }

    // Directed edges: along each lane, then lane end -> connector -> next lane start.
    for (const Lane& l : net.lanes) {
        for (size_t k = 0; k + 1 < l.segments.size(); ++k) {
            net.elements[static_cast<size_t>(l.segments[k])].next.push_back(l.segments[k + 1]);
        }
    }
    for (Element& e : net.elements) {
        if (!e.isConnector()) {
            continue;
        }
        const int last = net.lanes[static_cast<size_t>(e.fromLane)].segments.back();
        net.elements[static_cast<size_t>(last)].next.push_back(e.id);
        e.next.push_back(net.lanes[static_cast<size_t>(e.toLane)].segments.front());
    }
    for (const Element& e : net.elements) {
        for (int v : e.next) {
            net.elements[static_cast<size_t>(v)].prev.push_back(e.id);
        }
    }
    net.buildKerbs();
    return net;
}

double LaneNetwork::coarseClearance(double x, double y) const {
    if (general) {  // a district's kerb is exact: no conservative stand-in
        double nx = 0.0, ny = 0.0;
        return districtKerbDistance(*this, x, y, nx, ny);
    }
    const double hw = spec.laneWidth;  // half the carriageway: one lane each side of the centreline
    double best = -std::numeric_limits<double>::infinity();
    for (const Street& st : streets) {
        const double len = std::hypot(st.bx - st.ax, st.by - st.ay);
        const double ux = (st.bx - st.ax) / len, uy = (st.by - st.ay) / len;
        const double u = (x - st.ax) * ux + (y - st.ay) * uy;
        const double w = -(x - st.ax) * uy + (y - st.ay) * ux;
        const double u0 = st.a >= 0 ? -hw : 0.0;          // across the junction box at a junction end
        const double u1 = st.b >= 0 ? len + hw : len;     // a dead end stops at its end kerb
        best = std::max(best, std::min({u - u0, u1 - u, hw - std::fabs(w)}));
    }
    // Corner kerbs: between two perpendicular arms of a junction the kerb is an arc of the junction's radius.
    for (size_t j = 0; j < junctions.size(); ++j) {
        const Junction& jn = junctions[j];
        double arms[8][2];
        int n = 0;
        for (const Street& st : streets) {
            if (n < 8 && (st.a == static_cast<int>(j) || st.b == static_cast<int>(j))) {
                const double ox = st.a == static_cast<int>(j) ? st.bx : st.ax, oy = st.a == static_cast<int>(j) ? st.by : st.ay;
                const double len = std::hypot(ox - jn.x, oy - jn.y);
                arms[n][0] = (ox - jn.x) / len;
                arms[n][1] = (oy - jn.y) / len;
                ++n;
            }
        }
        const double r = kerbRadius[j];
        for (int i = 0; i < n; ++i) {
            for (int k = i + 1; k < n; ++k) {
                if (std::fabs(arms[i][0] * arms[k][0] + arms[i][1] * arms[k][1]) > 0.1) {
                    continue;  // not perpendicular neighbours
                }
                const double cx = jn.x + hw * (arms[i][0] + arms[k][0]), cy = jn.y + hw * (arms[i][1] + arms[k][1]);
                const double a = (x - cx) * arms[i][0] + (y - cy) * arms[i][1];
                const double b = (x - cx) * arms[k][0] + (y - cy) * arms[k][1];
                if (a >= 0.0 && b >= 0.0 && a <= r && b <= r) {
                    const double fx = cx + r * (arms[i][0] + arms[k][0]), fy = cy + r * (arms[i][1] + arms[k][1]);
                    best = std::max(best, std::hypot(x - fx, y - fy) - r);
                }
            }
        }
    }
    return best;
}

// Kerbs of a grid network. Each street contributes its two side kerbs. At a junction end, a side kerb
// stops where the corner arc begins if the junction has an arm on that side, and otherwise runs across
// the junction box to its far edge (T and L junctions). Each pair of perpendicular arms gets a corner arc
// of the junction's kerb radius; a dead end gets an end kerb.
void LaneNetwork::buildKerbs() {
    kerbLines.clear();
    kerbArcs.clear();
    const double hw = spec.laneWidth;
    auto armsOf = [&](int j) {
        std::vector<std::pair<double, double>> arms;
        for (const Street& st : streets) {
            if (st.a == j || st.b == j) {
                const double ox = st.a == j ? st.bx : st.ax, oy = st.a == j ? st.by : st.ay;
                const Junction& jn = junctions[static_cast<size_t>(j)];
                const double len = std::hypot(ox - jn.x, oy - jn.y);
                arms.push_back({(ox - jn.x) / len, (oy - jn.y) / len});
            }
        }
        return arms;
    };
    auto hasArm = [&](int j, double dx, double dy) {
        for (const auto& a : armsOf(j)) {
            if (a.first * dx + a.second * dy > 0.99) return true;
        }
        return false;
    };
    for (const Street& st : streets) {
        const double len = std::hypot(st.bx - st.ax, st.by - st.ay);
        const double ux = (st.bx - st.ax) / len, uy = (st.by - st.ay) / len;
        const double lx = -uy, ly = ux;  // left of a -> b
        for (double side : {1.0, -1.0}) {
            const double sx = side * lx, sy = side * ly;  // towards this kerb
            const double u0 = (st.a >= 0 && hasArm(st.a, sx, sy)) ? hw + kerbRadius[static_cast<size_t>(st.a)] : -hw;
            double u1 = len;
            if (st.b >= 0) {
                u1 = hasArm(st.b, sx, sy) ? len - hw - kerbRadius[static_cast<size_t>(st.b)] : len + hw;
            }
            KerbLine k;
            k.ax = st.ax + ux * u0 + sx * hw;
            k.ay = st.ay + uy * u0 + sy * hw;
            k.bx = st.ax + ux * u1 + sx * hw;
            k.by = st.ay + uy * u1 + sy * hw;
            k.nx = -sx;
            k.ny = -sy;
            kerbLines.push_back(k);
        }
        if (st.b < 0) {  // dead end: the end kerb, no turning head
            KerbLine e;
            e.ax = st.bx - lx * hw;
            e.ay = st.by - ly * hw;
            e.bx = st.bx + lx * hw;
            e.by = st.by + ly * hw;
            e.nx = -ux;
            e.ny = -uy;
            kerbLines.push_back(e);
        }
    }
    for (size_t j = 0; j < junctions.size(); ++j) {
        const auto arms = armsOf(static_cast<int>(j));
        const double r = kerbRadius[j];
        for (size_t i = 0; i < arms.size(); ++i) {
            for (size_t k = i + 1; k < arms.size(); ++k) {
                const double ax = arms[i].first + arms[k].first, ay = arms[i].second + arms[k].second;
                if (std::fabs(arms[i].first * arms[k].first + arms[i].second * arms[k].second) > 0.1) continue;
                KerbArc a;
                a.cx = junctions[j].x + (hw + r) * ax;
                a.cy = junctions[j].y + (hw + r) * ay;
                a.r = r;
                a.mid = std::atan2(-ay, -ax);
                a.half = 0.25 * kPi;
                kerbArcs.push_back(a);
            }
        }
    }
}

namespace {

// Nearest point of a kerb arc: distance, and the unit normal into the carriageway.
double arcDistance(const KerbArc& a, double x, double y, double& nx, double& ny) {
    const double dx = x - a.cx, dy = y - a.cy, rho = std::hypot(dx, dy);
    const double th = std::atan2(dy, dx);
    if (std::fabs(wrapAngle(th - a.mid)) <= a.half && rho > 1e-12) {
        nx = dx / rho;
        ny = dy / rho;
        return std::fabs(rho - a.r);
    }
    double best = std::numeric_limits<double>::infinity();
    for (double s : {-1.0, 1.0}) {
        const double ex = a.cx + a.r * std::cos(a.mid + s * a.half), ey = a.cy + a.r * std::sin(a.mid + s * a.half);
        const double d = std::hypot(x - ex, y - ey);
        if (d < best) {
            best = d;
            nx = (ex - a.cx) / a.r;  // the radial direction at the arc's end
            ny = (ey - a.cy) / a.r;
        }
    }
    return best;
}

double lineDistance(const KerbLine& k, double x, double y) {
    const double vx = k.bx - k.ax, vy = k.by - k.ay, l2 = vx * vx + vy * vy;
    const double t = l2 > 0.0 ? std::min(1.0, std::max(0.0, ((x - k.ax) * vx + (y - k.ay) * vy) / l2)) : 0.0;
    return std::hypot(x - (k.ax + t * vx), y - (k.ay + t * vy));
}

}  // namespace

double LaneNetwork::kerbDistance(double x, double y, double& nx, double& ny) const {
    if (general) {
        return districtKerbDistance(*this, x, y, nx, ny);
    }
    double best = std::numeric_limits<double>::infinity();
    nx = 0.0;
    ny = 0.0;
    for (const KerbLine& k : kerbLines) {
        const double d = lineDistance(k, x, y);
        if (d < best) {
            best = d;
            nx = k.nx;
            ny = k.ny;
        }
    }
    for (const KerbArc& a : kerbArcs) {
        double ax = 0.0, ay = 0.0;
        const double d = arcDistance(a, x, y, ax, ay);
        if (d < best) {
            best = d;
            nx = ax;
            ny = ay;
        }
    }
    return coarseClearance(x, y) >= 0.0 ? best : -best;
}

double LaneNetwork::roadClearance(double x, double y) const {
    double nx = 0.0, ny = 0.0;
    return kerbDistance(x, y, nx, ny);
}

double LaneNetwork::rayToKerb(double x, double y, double dx, double dy, double maxRange) const {
    double best = std::numeric_limits<double>::infinity();
    for (const KerbLine& k : kerbLines) {
        if (k.boundary) continue;  // a district's patch edge: not a kerb, nothing to see
        // x + t d = a + s (b - a), 0 <= s <= 1, t > 0
        const double ex = k.bx - k.ax, ey = k.by - k.ay;
        const double den = dx * ey - dy * ex;
        if (std::fabs(den) < 1e-12) continue;
        const double wx = k.ax - x, wy = k.ay - y;
        const double t = (wx * ey - wy * ex) / den;
        const double s = (wx * dy - wy * dx) / den;
        if (t > 1e-9 && s >= -1e-9 && s <= 1.0 + 1e-9 && t < best) best = t;
    }
    for (const KerbArc& a : kerbArcs) {
        const double px = x - a.cx, py = y - a.cy;
        const double b = px * dx + py * dy, c = px * px + py * py - a.r * a.r, disc = b * b - c;
        if (disc < 0.0) continue;
        const double sq = std::sqrt(disc);
        for (double t : {-b - sq, -b + sq}) {
            if (t <= 1e-9 || t >= best) continue;
            const double th = std::atan2(py + t * dy, px + t * dx);
            if (std::fabs(wrapAngle(th - a.mid)) <= a.half) best = t;
        }
    }
    return best <= maxRange ? best : -1.0;
}

double footprintClearance(const LaneNetwork& net, double x, double y, double yaw, double rearOverhang, double frontOffset,
                          double width) {
    const double c = std::cos(yaw), s = std::sin(yaw), hw = 0.5 * width;
    double best = std::numeric_limits<double>::infinity();
    auto eval = [&](double u, double w) { best = std::min(best, net.roadClearance(x + c * u - s * w, y + s * u + c * w)); };
    for (int i = 0; i <= 8; ++i) {
        const double u = -rearOverhang + (frontOffset + rearOverhang) * i / 8.0;
        eval(u, -hw);
        eval(u, hw);
    }
    for (int i = 1; i < 4; ++i) {
        const double w = -hw + width * i / 4.0;
        eval(-rearOverhang, w);
        eval(frontOffset, w);
    }
    return best;
}

LaneNetwork mirrored(const LaneNetwork& net) {
    LaneNetwork m = net;
    for (Junction& j : m.junctions) {
        j.x = -j.x;
    }
    for (Street& st : m.streets) {
        st.ax = -st.ax;
        st.bx = -st.bx;
    }
    for (Lane& l : m.lanes) {
        l.x0 = -l.x0;
        l.x1 = -l.x1;
        l.dirX = -l.dirX;
        l.centreX0 = -l.centreX0;
    }
    for (Element& e : m.elements) {
        for (PathPoint& p : e.pts) {
            p.x = -p.x;
            p.yaw = wrapAngle(kPi - p.yaw);
            p.kappa = -p.kappa;
        }
    }
    m.buildKerbs();
    return m;
}

std::vector<std::string> LaneNetwork::validate(double unitsPerMetre) const {
    std::vector<std::string> err;
    const double bound = 10000.0 / unitsPerMetre;
    const double kMax = 0.20;  // vehicle planning curvature limit, 1/m
    for (const Lane& l : lanes) {
        const double side = l.dirX * (l.y0 - l.centreY0) - l.dirY * (l.x0 - l.centreX0);  // > 0: left of its direction
        if (!spec.rightHand && side <= 0.0) {
            err.push_back("lane " + l.name + " is not left of its carriageway centre (not left-hand traffic)");
        }
        if (spec.rightHand && side >= 0.0) {
            err.push_back("lane " + l.name + " is not right of its carriageway centre (not right-hand traffic)");
        }
        if (l.length <= 0.0) {
            err.push_back("lane " + l.name + " has no length");
        }
    }
    for (const Element& e : elements) {
        if (e.name.find("geometry-misfit") != std::string::npos) {
            err.push_back("connector " + e.name + " does not fit its junction");
        }
        for (const PathPoint& p : e.pts) {
            if (std::fabs(p.x) > bound || std::fabs(p.y) > bound) {
                err.push_back("element " + e.name + " lies outside the engine's coordinate bound");
                break;
            }
        }
        std::set<int> unique(e.next.begin(), e.next.end());
        if (unique.size() != e.next.size()) {
            err.push_back("element " + e.name + " has parallel edges");
        }
        if (unique.count(e.id) != 0) {
            err.push_back("element " + e.name + " has a self-loop");
        }
        for (int v : e.next) {
            const PathPoint& a = e.pts.back();
            const PathPoint& b = elements[static_cast<size_t>(v)].pts.front();
            if (std::hypot(a.x - b.x, a.y - b.y) > 1e-6 || std::fabs(wrapAngle(a.yaw - b.yaw)) > 1e-6 ||
                std::fabs(a.kappa - b.kappa) > 1e-9) {
                err.push_back("join " + e.name + " -> " + elements[static_cast<size_t>(v)].name + " is not continuous");
            }
        }
        if (e.isConnector()) {
            const double turned = wrapAngle(e.pts.back().yaw - e.pts.front().yaw);
            // The grids' junctions are square; a real junction's turn may be a little sharper than a right angle.
            if (std::fabs(turned) > (general ? 0.75 : 0.5) * kPi + 1e-6) {
                err.push_back("connector " + e.name + " reverses direction");
            }
            double kArc = 0.0;
            for (const PathPoint& p : e.pts) {
                kArc = std::max(kArc, std::fabs(p.kappa));
            }
            if (kArc > kMax) {
                err.push_back("connector " + e.name + " exceeds the curvature limit");
            }
            const double maxRate = (kArc > 0.0 ? kArc / spec.clothoid : 0.0) * (1.0 + 1e-6) + 1e-9;
            for (size_t i = 1; i < e.pts.size(); ++i) {
                const double ds = e.pts[i].s - e.pts[i - 1].s;
                if (std::fabs(e.pts[i].kappa - e.pts[i - 1].kappa) > maxRate * ds + 1e-9) {
                    err.push_back("connector " + e.name + " has a curvature jump");
                    break;
                }
            }
        }
    }
    for (const BannedTurn& bt : banned) {
        const int a = lane(bt.fromLane), b = lane(bt.toLane);
        if (a >= 0 && b >= 0 && connector(a, b) >= 0) {
            err.push_back("banned turn " + bt.fromLane + " -> " + bt.toLane + " has a connector");
        }
    }
    if (general) {
        const std::vector<std::string> more = validateDistrict(*this);
        err.insert(err.end(), more.begin(), more.end());
    }
    return err;
}

double distanceToElement(const Element& e, double x, double y, double* s, double* lateral) {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < e.pts.size(); ++i) {
        const PathPoint& a = e.pts[i];
        const PathPoint& b = e.pts[i + 1];
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double len2 = dx * dx + dy * dy;
        double t = len2 > 0.0 ? ((x - a.x) * dx + (y - a.y) * dy) / len2 : 0.0;
        t = std::min(1.0, std::max(0.0, t));
        const double qx = a.x + dx * t, qy = a.y + dy * t;
        const double d = std::hypot(x - qx, y - qy);
        if (d < best) {
            best = d;
            if (s != nullptr) {
                *s = a.s + (b.s - a.s) * t;
            }
            if (lateral != nullptr) {
                const double len = std::sqrt(len2);
                *lateral = len > 0.0 ? (dx * (y - qy) - dy * (x - qx)) / len : 0.0;
            }
        }
    }
    return best;
}

RefRoute referenceRoute(const LaneNetwork& net, int start, int goal, const std::vector<int>& blocked) {
    const size_t n = net.elements.size();
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> dist(n, inf);
    std::vector<int> prev(n, -1);
    std::vector<char> isBlocked(n, 0);
    for (int b : blocked) {
        if (b != start) {
            isBlocked[static_cast<size_t>(b)] = 1;
        }
    }
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
    dist[static_cast<size_t>(start)] = 0.0;
    pq.push({0.0, start});
    while (!pq.empty()) {
        const Item top = pq.top();
        pq.pop();
        const int u = top.second;
        if (top.first > dist[static_cast<size_t>(u)]) {
            continue;
        }
        if (u == goal) {
            break;
        }
        for (int v : net.elements[static_cast<size_t>(u)].next) {
            if (isBlocked[static_cast<size_t>(v)] != 0) {
                continue;
            }
            const double nd = top.first + net.cost(v);
            if (nd < dist[static_cast<size_t>(v)]) {
                dist[static_cast<size_t>(v)] = nd;
                prev[static_cast<size_t>(v)] = u;
                pq.push({nd, v});
            }
        }
    }
    RefRoute r;
    if (dist[static_cast<size_t>(goal)] == inf) {
        return r;
    }
    for (int v = goal; v != -1; v = prev[static_cast<size_t>(v)]) {
        r.nodes.push_back(v);
        if (v == start) {
            break;
        }
    }
    std::reverse(r.nodes.begin(), r.nodes.end());
    r.cost = dist[static_cast<size_t>(goal)];
    r.found = true;
    return r;
}

double routeCost(const LaneNetwork& net, const std::vector<int>& route) {
    double c = 0.0;
    for (size_t i = 1; i < route.size(); ++i) {
        c += net.cost(route[i]);
    }
    return c;
}

bool routeLegal(const LaneNetwork& net, const std::vector<int>& route) {
    for (size_t i = 1; i < route.size(); ++i) {
        const std::vector<int>& nx = net.elements[static_cast<size_t>(route[i - 1])].next;
        if (std::find(nx.begin(), nx.end(), route[i]) == nx.end()) {
            return false;
        }
    }
    return true;
}

int associate(const LaneNetwork& net, double x, double y, double yaw, double* distance) {
    int best = -1;
    double bestD = std::numeric_limits<double>::infinity();
    for (const Element& e : net.elements) {
        for (size_t i = 0; i + 1 < e.pts.size(); ++i) {
            const PathPoint& a = e.pts[i];
            if (std::fabs(wrapAngle(yaw - a.yaw)) > 0.25 * kPi) {
                continue;
            }
            const PathPoint& b = e.pts[i + 1];
            const double dx = b.x - a.x, dy = b.y - a.y;
            const double len2 = dx * dx + dy * dy;
            double t = len2 > 0.0 ? ((x - a.x) * dx + (y - a.y) * dy) / len2 : 0.0;
            t = std::min(1.0, std::max(0.0, t));
            const double d = std::hypot(x - (a.x + dx * t), y - (a.y + dy * t));
            if (d < bestD) {
                bestD = d;
                best = e.id;
            }
        }
    }
    if (distance != nullptr) {
        *distance = bestD;
    }
    return best;
}

std::string routeText(const LaneNetwork& net, const std::vector<int>& route) {
    std::ostringstream os;
    for (size_t i = 0; i < route.size(); ++i) {
        os << (i ? " " : "") << net.elements[static_cast<size_t>(route[i])].name;
    }
    return os.str();
}

}  // namespace road

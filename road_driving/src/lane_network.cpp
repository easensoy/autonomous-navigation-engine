#include "road_driving/lane_network.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <set>
#include <sstream>
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

LaneNetwork buildR1(const NetworkSpec& spec) {
    LaneNetwork net;
    net.spec = spec;
    const double xs[3] = {0.0, 120.0, 240.0};
    const double ys[3] = {0.0, 80.0, 160.0};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            net.junctions.push_back({"J" + std::to_string(c) + std::to_string(r), xs[c], ys[r]});
        }
    }
    auto J = [&](int c, int r) {
        const int j = r * 3 + c;
        return EndPoint{net.junctions[static_cast<size_t>(j)].name, xs[c], ys[r], j};
    };
    const EndPoint deadEnd{"D21", 300.0, 80.0, -1};

    struct Street {
        EndPoint a, b;
        bool twoWay;
    };
    std::vector<Street> streets;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 2; ++c) {
            streets.push_back({J(c, r), J(c + 1, r), r != 1});  // r = 1: one-way eastbound (a -> b)
        }
    }
    for (int c = 0; c < 3; ++c) {
        for (int r = 0; r < 2; ++r) {
            streets.push_back({J(c, r), J(c, r + 1), true});
        }
    }
    streets.push_back({J(2, 1), deadEnd, true});  // cul-de-sac, no turning head

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
    for (const Street& st : streets) {
        addLane(st.a, st.b);
        if (st.twoWay) {
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

    // Turning movements. U-turns do not exist; the banned turn is listed and never built.
    net.banned.push_back({"J11", "J10>J11", "J11>J21"});
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
                    const double radius = e.turn == Turn::Left ? spec.leftRadius : spec.rightRadius;
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
    return net;
}

LaneNetwork mirrored(const LaneNetwork& net) {
    LaneNetwork m = net;
    for (Junction& j : m.junctions) {
        j.x = -j.x;
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
    return m;
}

std::vector<std::string> LaneNetwork::validate(double unitsPerMetre) const {
    std::vector<std::string> err;
    const double bound = 10000.0 / unitsPerMetre;
    const double kMax = 0.20;  // vehicle planning curvature limit, 1/m
    for (const Lane& l : lanes) {
        const double side = l.dirX * (l.y0 - l.centreY0) - l.dirY * (l.x0 - l.centreX0);
        if (side <= 0.0) {
            err.push_back("lane " + l.name + " is not left of its carriageway centre (not left-hand traffic)");
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
            if (std::fabs(turned) > 0.5 * kPi + 1e-6) {
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

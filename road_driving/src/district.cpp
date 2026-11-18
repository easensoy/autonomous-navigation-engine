#include "road_driving/district.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

namespace road {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

struct P {
    double x = 0.0, y = 0.0;
};
P operator+(P a, P b) { return {a.x + b.x, a.y + b.y}; }
P operator-(P a, P b) { return {a.x - b.x, a.y - b.y}; }
P operator*(P a, double k) { return {a.x * k, a.y * k}; }
double dot(P a, P b) { return a.x * b.x + a.y * b.y; }
double cross(P a, P b) { return a.x * b.y - a.y * b.x; }
P leftOf(P u) { return {-u.y, u.x}; }
double norm(P a) { return std::hypot(a.x, a.y); }
P unit(P a) {
    const double n = norm(a);
    return {a.x / n, a.y / n};
}

// The grids' sampler (lane_network.cpp), repeated so that their code stays as it was: consecutive
// linear-curvature pieces from a start pose, at most `step` apart; heading exact, position integrated with
// 16 midpoint substeps per sample.
struct Piece {
    double length, k0, k1;
};

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

std::vector<PathPoint> straight(P a, P b, double step) {
    const double len = norm(b - a);
    const double yaw = std::atan2(b.y - a.y, b.x - a.x);
    const int n = std::max(1, static_cast<int>(std::ceil(len / step - 1e-9)));
    std::vector<PathPoint> out;
    out.reserve(static_cast<size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        out.push_back({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, yaw, 0.0, len * t});
    }
    out.back().x = b.x;
    out.back().y = b.y;
    return out;
}

// A symmetric clothoid-arc-clothoid turning by delta with arc radius R: the distance from either end to the
// intersection of its end tangents.
double tangentLength(double delta, double R, double ls, double step) {
    const double k = (delta > 0.0 ? 1.0 : -1.0) / R;
    const double la = std::max(0.0, std::fabs(delta) * R - ls);
    const std::vector<PathPoint> pts = samplePieces(0.0, 0.0, 0.0, {{ls, 0.0, k}, {la, k, k}, {ls, k, 0.0}}, step);
    return pts.back().x - pts.back().y / std::tan(delta);
}

// The largest radius in [rMin, rPref] whose tangent length is at most `room`; -1 if even rMin needs more.
double fitRadius(double delta, double rMin, double rPref, double room, double ls, double step) {
    if (tangentLength(delta, rMin, ls, step) > room + 1e-9) {
        return -1.0;
    }
    if (tangentLength(delta, rPref, ls, step) <= room) {
        return rPref;
    }
    double lo = rMin, hi = rPref;
    for (int i = 0; i < 40; ++i) {
        const double mid = 0.5 * (lo + hi);
        (tangentLength(delta, mid, ls, step) <= room ? lo : hi) = mid;
    }
    return lo;
}

struct Arm {
    int street = -1;
    bool atA = true;  // the junction is the street's a end
    P u;              // unit direction away from the junction
    double angle = 0.0;
    double wl = 0.0, wr = 0.0;  // the carriageway's width on the arm's left and right of its centreline
    double cornerL = 0.0, cornerR = 0.0;  // where the arm's left and right kerbs start, from the junction
    double mouth = 0.0;
};

std::string before(const std::string& s, char c) { return s.substr(0, s.find(c)); }
std::string after(const std::string& s, char c) { return s.substr(s.find(c) + 1); }

}  // namespace

std::vector<std::string> districtNames() { return {"KEN", "KNB"}; }

bool findDistrict(const std::string& name, DistrictSpec& out) {
    if (name == "KEN") {
        out = districtKensington();
        return true;
    }
    if (name == "KNB") {
        out = districtKnightsbridge();
        return true;
    }
    return false;
}

LaneNetwork buildDistrict(const DistrictSpec& d, const NetworkSpec& baseSpec) {
    NetworkSpec spec = baseSpec;
    spec.name = d.id;
    spec.laneWidth = 3.2;
    spec.mouth = 8.0;  // a typical value, for display; each junction arm sets its own
    LaneNetwork net;
    net.spec = spec;
    net.general = true;
    const double ls = spec.clothoid;

    std::map<std::string, size_t> jIndex;
    for (size_t i = 0; i < d.junctions.size(); ++i) {
        if (!jIndex.emplace(d.junctions[i].name, i).second) {
            throw std::invalid_argument("district " + d.id + ": junction " + d.junctions[i].name + " is defined twice");
        }
    }
    std::vector<int> netIndex(d.junctions.size(), -1);  // -1: a patch edge
    for (size_t i = 0; i < d.junctions.size(); ++i) {
        const DistrictJunction& jn = d.junctions[i];
        if (!jn.open) {
            netIndex[i] = static_cast<int>(net.junctions.size());
            net.junctions.push_back({jn.name, jn.x, jn.y});
            net.kerbRadius.push_back(jn.kerbRadius);
        }
    }

    const size_t ns = d.streets.size();
    std::vector<P> A(ns), B(ns), U(ns);
    std::vector<double> L(ns);
    std::vector<size_t> JA(ns), JB(ns);
    std::vector<std::vector<Arm>> arms(d.junctions.size());
    for (size_t i = 0; i < ns; ++i) {
        const DistrictStreet& st = d.streets[i];
        const auto ia = jIndex.find(st.a), ib = jIndex.find(st.b);
        if (ia == jIndex.end() || ib == jIndex.end()) {
            throw std::invalid_argument("district " + d.id + ": street " + st.name + " names an unknown junction");
        }
        JA[i] = ia->second;
        JB[i] = ib->second;
        A[i] = {d.junctions[JA[i]].x, d.junctions[JA[i]].y};
        B[i] = {d.junctions[JB[i]].x, d.junctions[JB[i]].y};
        L[i] = norm(B[i] - A[i]);
        U[i] = (B[i] - A[i]) * (1.0 / L[i]);
        Arm a;
        a.street = static_cast<int>(i);
        a.atA = true;
        a.u = U[i];
        a.angle = std::atan2(a.u.y, a.u.x);
        a.wl = st.halfWidth;
        a.wr = st.right();
        arms[JA[i]].push_back(a);
        Arm b = a;
        b.atA = false;
        b.u = U[i] * -1.0;
        b.angle = std::atan2(b.u.y, b.u.x);
        b.wl = st.right();  // seen from b, a -> b's right is on the left
        b.wr = st.halfWidth;
        arms[JB[i]].push_back(b);
        Street s;
        s.a = netIndex[JA[i]];
        s.b = netIndex[JB[i]];
        s.ax = A[i].x;
        s.ay = A[i].y;
        s.bx = B[i].x;
        s.by = B[i].y;
        s.name = st.name;
        s.halfWidth = st.halfWidth;
        s.halfWidthRight = st.right();
        s.median = st.median;
        s.laneOffset = st.laneOffset;
        net.streets.push_back(s);
    }

    // Corner kerbs. Around each junction, between neighbouring arms: a fillet of the junction's kerb radius
    // where the block corner is convex, a mitre where it is reflex, and a straight run (with a step if the
    // widths differ) where the arms are opposite.
    for (size_t j = 0; j < d.junctions.size(); ++j) {
        std::vector<Arm>& as = arms[j];
        const DistrictJunction& jn = d.junctions[j];
        if (jn.open) {
            if (as.size() != 1) {
                throw std::invalid_argument("district " + d.id + ": patch edge " + jn.name + " must end exactly one street");
            }
            continue;
        }
        if (as.size() < 2) {
            throw std::invalid_argument("district " + d.id + ": junction " + jn.name + " has one street; mark it as a patch edge");
        }
        std::sort(as.begin(), as.end(), [](const Arm& p, const Arm& q) { return p.angle < q.angle; });
        const P c{jn.x, jn.y};
        const size_t n = as.size();
        for (size_t k = 0; k < n; ++k) {
            Arm& a = as[k];
            Arm& b = as[(k + 1) % n];
            double theta = b.angle - a.angle;
            while (theta <= 1e-12) theta += 2.0 * kPi;
            const P pa = c + leftOf(a.u) * a.wl;  // on a's left kerb
            const P pb = c - leftOf(b.u) * b.wr;  // on b's right kerb
            if (theta < 1e-6 || theta > 2.0 * kPi - 1e-6) {
                throw std::invalid_argument("district " + d.id + ": two streets leave junction " + jn.name + " in the same direction");
            }
            if (std::fabs(std::sin(theta)) < 1e-9) {
                // Exactly opposite arms: the kerb runs straight across, with a step where the widths differ. The
                // step continues the kerb chain from a's kerb to b's, so the carriageway is on its left.
                a.cornerL = 0.0;
                b.cornerR = 0.0;
                if (norm(pb - pa) > 1e-9) {
                    KerbLine k2;
                    k2.ax = pa.x;
                    k2.ay = pa.y;
                    k2.bx = pb.x;
                    k2.by = pb.y;
                    const P nrm = unit(leftOf(pb - pa));
                    k2.nx = nrm.x;
                    k2.ny = nrm.y;
                    net.kerbLines.push_back(k2);
                }
                continue;
            }
            const P r = pb - pa;
            const double den = cross(a.u, b.u);
            const double t = cross(r, b.u) / den;
            const double s = cross(r, a.u) / den;
            const P X = pa + a.u * t;
            if (theta < kPi) {
                const double rad = jn.kerbRadius;
                const double tt = rad / std::tan(0.5 * theta);
                a.cornerL = t + tt;
                b.cornerR = s + tt;
                const P C = X + unit(a.u + b.u) * (rad / std::sin(0.5 * theta));
                KerbArc arc;
                arc.cx = C.x;
                arc.cy = C.y;
                arc.r = rad;
                arc.mid = std::atan2(X.y - C.y, X.x - C.x);
                arc.half = 0.5 * (kPi - theta);
                net.kerbArcs.push_back(arc);
            } else {
                a.cornerL = t;
                b.cornerR = s;
            }
        }
    }

    auto armOf = [&](size_t junction, size_t street, bool atA) -> Arm& {
        for (Arm& a : arms[junction]) {
            if (a.street == static_cast<int>(street) && a.atA == atA) return a;
        }
        throw std::logic_error("district arm not found");
    };

    // Mouths: lanes end clear of the corner kerbs, and at a bend early enough for a smooth curve.
    for (size_t j = 0; j < d.junctions.size(); ++j) {
        const DistrictJunction& jn = d.junctions[j];
        std::vector<Arm>& as = arms[j];
        if (jn.open) {
            as[0].mouth = 6.0;  // lanes end where the car's front stays clear of the patch edge
            continue;
        }
        double corners = 0.0;
        for (const Arm& a : as) corners = std::max({corners, a.cornerL, a.cornerR});
        if (jn.mouth > 0.0) {
            for (Arm& a : as) a.mouth = jn.mouth;
            continue;
        }
        if (as.size() == 2) {
            const double delta = kPi - std::acos(std::max(-1.0, std::min(1.0, dot(as[0].u, as[1].u))));
            double mouth = corners + 0.5;
            if (delta > 1e-4) {
                const double off = std::max(d.streets[static_cast<size_t>(as[0].street)].laneOffset, d.streets[static_cast<size_t>(as[1].street)].laneOffset);
                const double cap = 0.45 * std::min(L[static_cast<size_t>(as[0].street)], L[static_cast<size_t>(as[1].street)]);
                const double rMin = std::max(5.5, ls / delta) + 1e-9;
                const double rPref = std::max(rMin, delta < 0.35 ? d.bendRadius : d.rightRadius);
                const double extra = off * std::tan(0.5 * delta) + 0.3;
                double R = fitRadius(delta, rMin, rPref, cap - extra, ls, spec.sampleStep);
                if (R < 0.0) R = rMin;
                mouth = std::max(mouth, tangentLength(delta, R, ls, spec.sampleStep) + extra);
            }
            for (Arm& a : as) a.mouth = mouth;
            continue;
        }
        for (Arm& a : as) a.mouth = std::max(7.0, std::max(a.cornerL, a.cornerR) + 1.5);
    }

    // Lanes, left of each street's centreline in the direction of travel.
    for (size_t i = 0; i < ns; ++i) {
        const DistrictStreet& st = d.streets[i];
        const Arm& aa = armOf(JA[i], i, true);
        const Arm& ab = armOf(JB[i], i, false);
        if (L[i] - aa.mouth - ab.mouth < 1.0) {
            throw std::invalid_argument("district " + d.id + ": street " + st.name + " (" + st.a + "-" + st.b + ") is too short for its junctions");
        }
        auto addLane = [&](bool forward) {
            Lane l;
            const size_t jf = forward ? JA[i] : JB[i], jt = forward ? JB[i] : JA[i];
            l.name = d.junctions[jf].name + ">" + d.junctions[jt].name;
            l.from = netIndex[jf];
            l.to = netIndex[jt];
            const P dir = forward ? U[i] : U[i] * -1.0;
            const P start = forward ? A[i] : B[i];
            const double m0 = forward ? aa.mouth : ab.mouth, m1 = forward ? ab.mouth : aa.mouth;
            const P nl = leftOf(dir);
            const P c0 = start + dir * m0;
            const P p0 = c0 + nl * st.laneOffset;
            const P p1 = start + dir * (L[i] - m1) + nl * st.laneOffset;
            l.dirX = dir.x;
            l.dirY = dir.y;
            l.centreX0 = c0.x;
            l.centreY0 = c0.y;
            l.x0 = p0.x;
            l.y0 = p0.y;
            l.x1 = p1.x;
            l.y1 = p1.y;
            l.length = L[i] - m0 - m1;
            net.lanes.push_back(l);
        };
        if (st.oneWay >= 0) addLane(true);
        if (st.oneWay <= 0) addLane(false);
    }

    // Lane segments, as on the grids.
    for (size_t li = 0; li < net.lanes.size(); ++li) {
        Lane& l = net.lanes[li];
        const int n = std::max(1, static_cast<int>(std::ceil(l.length / spec.maxSegment - 1e-9)));
        std::vector<P> e(static_cast<size_t>(n) + 1);
        for (int k = 0; k <= n; ++k) {
            const double s = l.length * k / n;
            e[static_cast<size_t>(k)] = {l.x0 + l.dirX * s, l.y0 + l.dirY * s};
        }
        e.back() = {l.x1, l.y1};
        for (int k = 0; k < n; ++k) {
            Element el;
            el.id = static_cast<int>(net.elements.size());
            el.kind = ElementKind::LaneSegment;
            el.name = l.name + "/" + std::to_string(k);
            el.lane = static_cast<int>(li);
            el.segment = k;
            el.laneS0 = l.length * k / n;
            el.speedLimit = spec.speedLimit;
            el.pts = straight(e[static_cast<size_t>(k)], e[static_cast<size_t>(k) + 1], spec.sampleStep);
            el.length = el.pts.back().s;
            l.segments.push_back(el.id);
            net.elements.push_back(el);
        }
    }

    // Turning movements: a symmetric clothoid-arc-clothoid on the intersection of the two lanes' lines, with
    // the preferred radius where it fits and the largest smaller one where it does not. No U-turns; banned
    // turns are listed and never built.
    net.banned = d.banned;
    for (size_t j = 0; j < d.junctions.size(); ++j) {
        const DistrictJunction& jn = d.junctions[j];
        if (jn.open) continue;
        const int nj = netIndex[j];
        const bool bend = arms[j].size() == 2;
        for (size_t in = 0; in < net.lanes.size(); ++in) {
            if (net.lanes[in].to != nj) continue;
            for (size_t out = 0; out < net.lanes.size(); ++out) {
                if (net.lanes[out].from != nj) continue;
                const Lane& li = net.lanes[in];
                const Lane& lo = net.lanes[out];
                const double yin = std::atan2(li.dirY, li.dirX), yout = std::atan2(lo.dirY, lo.dirX);
                const double delta = wrapAngle(yout - yin);
                if (std::fabs(delta) > 0.75 * kPi) continue;  // a U-turn: not a movement
                bool isBanned = false;
                for (const BannedTurn& bt : net.banned) {
                    isBanned = isBanned || (bt.junction == jn.name && bt.fromLane == li.name && bt.toLane == lo.name);
                }
                if (isBanned) continue;
                Element e;
                e.id = static_cast<int>(net.elements.size());
                e.kind = ElementKind::Connector;
                e.junction = nj;
                e.fromLane = static_cast<int>(in);
                e.toLane = static_cast<int>(out);
                e.speedLimit = spec.speedLimit;
                e.name = jn.name + ":" + before(li.name, '>') + ">" + after(lo.name, '>');
                const P pin{li.x1, li.y1}, pout{lo.x0, lo.y0};
                const P uin{li.dirX, li.dirY}, uout{lo.dirX, lo.dirY};
                bool misfit = false;
                if (std::fabs(delta) < 1e-4) {
                    e.turn = Turn::Ahead;
                    misfit = std::fabs(cross(uin, pout - pin)) > 1e-3 || dot(pout - pin, uin) <= 0.0;
                    e.pts = straight(pin, pout, spec.sampleStep);
                } else {
                    // A bend in a street (two arms) is no choice of turn: ahead unless it is sharp.
                    const bool ahead = std::fabs(delta) < (bend ? 0.35 : 0.1);
                    e.turn = ahead ? Turn::Ahead : (delta > 0.0 ? Turn::Left : Turn::Right);
                    const double den = cross(uin, uout);
                    const P r = pout - pin;
                    const double din = cross(r, uout) / den, dout = cross(uin, r) / den;
                    const double rMin = std::max(5.5, ls / std::fabs(delta)) + 1e-9;
                    const double pref = std::fabs(delta) < 0.35 ? d.bendRadius : (delta > 0.0 ? d.leftRadius : d.rightRadius);
                    const double R = din > 0.0 && dout > 0.0 ? fitRadius(delta, rMin, std::max(rMin, bend && std::fabs(delta) < 0.35 ? d.bendRadius : pref),
                                                                         std::min(din, dout), ls, spec.sampleStep)
                                                             : -1.0;
                    if (R < 0.0) {
                        misfit = true;
                        e.pts = straight(pin, pout, spec.sampleStep);
                    } else {
                        const double T = tangentLength(delta, R, ls, spec.sampleStep);
                        const double k = (delta > 0.0 ? 1.0 : -1.0) / R;
                        const double la = std::max(0.0, std::fabs(delta) * R - ls);
                        e.pts = samplePieces(pin.x, pin.y, yin, {{din - T, 0.0, 0.0}, {ls, 0.0, k}, {la, k, k}, {ls, k, 0.0}, {dout - T, 0.0, 0.0}},
                                             spec.sampleStep);
                        misfit = std::hypot(e.pts.back().x - pout.x, e.pts.back().y - pout.y) > 1e-6;
                        e.pts.back().x = pout.x;
                        e.pts.back().y = pout.y;
                    }
                }
                if (misfit) e.name += "(geometry-misfit)";  // reported by validate()
                e.length = e.pts.back().s;
                net.elements.push_back(e);
            }
        }
    }

    // Directed edges, as on the grids.
    for (const Lane& l : net.lanes) {
        for (size_t k = 0; k + 1 < l.segments.size(); ++k) {
            net.elements[static_cast<size_t>(l.segments[k])].next.push_back(l.segments[k + 1]);
        }
    }
    for (Element& e : net.elements) {
        if (!e.isConnector()) continue;
        const int last = net.lanes[static_cast<size_t>(e.fromLane)].segments.back();
        net.elements[static_cast<size_t>(last)].next.push_back(e.id);
        e.next.push_back(net.lanes[static_cast<size_t>(e.toLane)].segments.front());
    }
    for (const Element& e : net.elements) {
        for (int v : e.next) {
            net.elements[static_cast<size_t>(v)].prev.push_back(e.id);
        }
    }

    // Street kerbs, central reservations and patch edges.
    auto addLine = [&](P a, P b, P n, bool boundary) {
        KerbLine k;
        k.ax = a.x;
        k.ay = a.y;
        k.bx = b.x;
        k.by = b.y;
        k.nx = n.x;
        k.ny = n.y;
        k.boundary = boundary;
        net.kerbLines.push_back(k);
    };
    for (size_t i = 0; i < ns; ++i) {
        const DistrictStreet& st = d.streets[i];
        const Arm& aa = armOf(JA[i], i, true);
        const Arm& ab = armOf(JB[i], i, false);
        const P n = leftOf(U[i]);
        const double wl = st.halfWidth, wr = st.right();
        auto at = [&](double s, double off) { return A[i] + (B[i] - A[i]) * (s / L[i]) + n * off; };
        const double l0 = aa.cornerL, l1 = L[i] - ab.cornerR, r0 = aa.cornerR, r1 = L[i] - ab.cornerL;
        if (l1 - l0 < 0.5 || r1 - r0 < 0.5) {
            throw std::invalid_argument("district " + d.id + ": street " + st.name + " (" + st.a + "-" + st.b + ") is too short for its corner kerbs");
        }
        addLine(at(l0, wl), at(l1, wl), n * -1.0, false);
        addLine(at(r0, -wr), at(r1, -wr), n, false);
        if (d.junctions[JA[i]].open) addLine(at(0.0, -wr), at(0.0, wl), U[i], true);
        if (d.junctions[JB[i]].open) addLine(at(L[i], wl), at(L[i], -wr), U[i] * -1.0, true);
        if (st.median > 0.0) {
            const double wm = 0.5 * st.median;
            const double noseA = d.junctions[JA[i]].open ? 3.0 : aa.mouth + 1.0, noseB = d.junctions[JB[i]].open ? 3.0 : ab.mouth + 1.0;
            const double s0 = noseA + wm, s1 = L[i] - noseB - wm;
            if (s1 - s0 < 1.0) {
                throw std::invalid_argument("district " + d.id + ": street " + st.name + " is too short for its central reservation");
            }
            addLine(at(s0, wm), at(s1, wm), n, false);
            addLine(at(s0, -wm), at(s1, -wm), n * -1.0, false);
            for (int end = 0; end < 2; ++end) {
                const P cc = at(end == 0 ? s0 : s1, 0.0);
                const P dir = end == 0 ? U[i] * -1.0 : U[i];
                KerbArc nose;
                nose.cx = cc.x;
                nose.cy = cc.y;
                nose.r = wm;
                nose.mid = std::atan2(dir.y, dir.x);
                nose.half = 0.5 * kPi;
                net.kerbArcs.push_back(nose);
            }
        }
    }

    // Pseudo-normals where kerb pieces meet: every end must meet exactly one other.
    struct End {
        P p, n;
        int kind;  // 0 line, 1 arc
        size_t idx;
        int which;
    };
    std::vector<End> ends;
    for (size_t i = 0; i < net.kerbLines.size(); ++i) {
        const KerbLine& k = net.kerbLines[i];
        ends.push_back({{k.ax, k.ay}, {k.nx, k.ny}, 0, i, 0});
        ends.push_back({{k.bx, k.by}, {k.nx, k.ny}, 0, i, 1});
    }
    for (size_t i = 0; i < net.kerbArcs.size(); ++i) {
        const KerbArc& a = net.kerbArcs[i];
        for (int w = 0; w < 2; ++w) {
            const double th = a.mid + (w == 0 ? -a.half : a.half);
            ends.push_back({{a.cx + a.r * std::cos(th), a.cy + a.r * std::sin(th)}, {std::cos(th), std::sin(th)}, 1, i, w});
        }
    }
    for (size_t i = 0; i < ends.size(); ++i) {
        int partners = 0;
        P sum = ends[i].n;
        for (size_t k = 0; k < ends.size(); ++k) {
            if (k != i && norm(ends[k].p - ends[i].p) < 1e-6) {
                ++partners;
                sum = sum + ends[k].n;
            }
        }
        if (partners != 1 || norm(sum) < 1e-9) {
            throw std::invalid_argument("district " + d.id + ": the kerb does not close near (" + std::to_string(ends[i].p.x) + ", " +
                                        std::to_string(ends[i].p.y) + ")");
        }
        const P pn = unit(sum);
        if (ends[i].kind == 0) {
            KerbLine& k = net.kerbLines[ends[i].idx];
            (ends[i].which == 0 ? k.nax : k.nbx) = pn.x;
            (ends[i].which == 0 ? k.nay : k.nby) = pn.y;
        } else {
            KerbArc& a = net.kerbArcs[ends[i].idx];
            (ends[i].which == 0 ? a.n0x : a.n1x) = pn.x;
            (ends[i].which == 0 ? a.n0y : a.n1y) = pn.y;
        }
    }
    return net;
}

double districtKerbDistance(const LaneNetwork& net, double x, double y, double& nx, double& ny) {
    double best = kInf;
    double qx = x, qy = y, sx = 0.0, sy = 0.0;
    nx = 0.0;
    ny = 0.0;
    for (const KerbLine& k : net.kerbLines) {
        const double vx = k.bx - k.ax, vy = k.by - k.ay, l2 = vx * vx + vy * vy;
        const double t = l2 > 0.0 ? std::min(1.0, std::max(0.0, ((x - k.ax) * vx + (y - k.ay) * vy) / l2)) : 0.0;
        const double px = k.ax + t * vx, py = k.ay + t * vy;
        const double dd = std::hypot(x - px, y - py);
        if (dd < best) {
            best = dd;
            qx = px;
            qy = py;
            nx = k.nx;
            ny = k.ny;
            if (t <= 0.0) {
                sx = k.nax;
                sy = k.nay;
            } else if (t >= 1.0) {
                sx = k.nbx;
                sy = k.nby;
            } else {
                sx = k.nx;
                sy = k.ny;
            }
        }
    }
    for (const KerbArc& a : net.kerbArcs) {
        const double dx = x - a.cx, dy = y - a.cy, rho = std::hypot(dx, dy);
        const double th = std::atan2(dy, dx);
        if (std::fabs(wrapAngle(th - a.mid)) <= a.half && rho > 1e-12) {
            const double dd = std::fabs(rho - a.r);
            if (dd < best) {
                best = dd;
                nx = sx = dx / rho;
                ny = sy = dy / rho;
                qx = a.cx + a.r * nx;
                qy = a.cy + a.r * ny;
            }
            continue;
        }
        for (int w = 0; w < 2; ++w) {
            const double e = a.mid + (w == 0 ? -a.half : a.half);
            const double ex = a.cx + a.r * std::cos(e), ey = a.cy + a.r * std::sin(e);
            const double dd = std::hypot(x - ex, y - ey);
            if (dd < best) {
                best = dd;
                qx = ex;
                qy = ey;
                nx = std::cos(e);
                ny = std::sin(e);
                sx = w == 0 ? a.n0x : a.n1x;
                sy = w == 0 ? a.n0y : a.n1y;
            }
        }
    }
    return (x - qx) * sx + (y - qy) * sy >= 0.0 ? best : -best;
}

std::vector<std::vector<std::pair<double, double>>> carriagewayLoops(const LaneNetwork& net, double step) {
    // Each kerb piece as a polyline with the carriageway on its left.
    std::vector<std::vector<P>> prims;
    for (const KerbLine& k : net.kerbLines) {
        const P dir{k.ny, -k.nx};  // its left normal is the kerb's normal
        const P a{k.ax, k.ay}, b{k.bx, k.by};
        prims.push_back(dot(b - a, dir) >= 0.0 ? std::vector<P>{a, b} : std::vector<P>{b, a});
    }
    for (const KerbArc& a : net.kerbArcs) {
        const int n = std::max(2, static_cast<int>(std::ceil(2.0 * a.half / step)));
        std::vector<P> pts;
        for (int i = 0; i <= n; ++i) {  // clockwise: the carriageway, outside the circle, on the left
            const double th = a.mid + a.half - 2.0 * a.half * i / n;
            pts.push_back({a.cx + a.r * std::cos(th), a.cy + a.r * std::sin(th)});
        }
        prims.push_back(pts);
    }
    std::vector<std::vector<std::pair<double, double>>> loops;
    std::vector<char> used(prims.size(), 0);
    for (size_t first = 0; first < prims.size(); ++first) {
        if (used[first]) continue;
        used[first] = 1;
        std::vector<std::pair<double, double>> loop;
        size_t cur = first;
        for (size_t guard = 0; guard <= prims.size(); ++guard) {
            for (size_t i = 0; i + 1 < prims[cur].size(); ++i) loop.push_back({prims[cur][i].x, prims[cur][i].y});
            const P end = prims[cur].back();
            if (norm(end - prims[first].front()) < 1e-6) break;
            size_t next = prims.size();
            for (size_t k = 0; k < prims.size(); ++k) {
                if (!used[k] && norm(prims[k].front() - end) < 1e-6) {
                    next = k;
                    break;
                }
            }
            if (next == prims.size()) {
                loop.assign(1, {end.x, end.y});  // an open chain: validateDistrict reports where it stops
                break;
            }
            used[next] = 1;
            cur = next;
        }
        loops.push_back(loop);
    }
    return loops;
}

bool insideCarriageway(const std::vector<std::vector<std::pair<double, double>>>& loops, double x, double y) {
    bool inside = false;
    for (const auto& lp : loops) {
        const size_t n = lp.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            const double xi = lp[i].first, yi = lp[i].second, xj = lp[j].first, yj = lp[j].second;
            if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) inside = !inside;
        }
    }
    return inside;
}

std::vector<std::string> validateDistrict(const LaneNetwork& net) {
    std::vector<std::string> err;
    for (const auto& lp : carriagewayLoops(net)) {
        if (lp.size() < 3) {
            err.push_back("the kerb of " + net.spec.name + " does not form closed loops" +
                          (lp.empty() ? std::string() : " (it stops at " + std::to_string(lp[0].first) + ", " + std::to_string(lp[0].second) + ")"));
        }
    }
    double nx = 0.0, ny = 0.0;
    for (const Lane& l : net.lanes) {
        for (double f : {0.0, 0.5, 1.0}) {
            const double x = l.x0 + (l.x1 - l.x0) * f, y = l.y0 + (l.y1 - l.y0) * f;
            if (districtKerbDistance(net, x, y, nx, ny) < 1.0) {
                err.push_back("lane " + l.name + " is not on the carriageway with a metre to spare");
                break;
            }
        }
    }
    return err;
}

// ---- The districts ----------------------------------------------------------------------------------------
// Coordinates are metres east and north of the district's origin; positions of junctions are OpenStreetMap
// nodes (or, on Queen's Gate, the midpoint of the two carriageways' nodes) from the extract named in
// `source`. road_driving/viewer/districts/PROVENANCE.md has the full account, deviations included.

DistrictSpec districtKensington() {
    DistrictSpec d;
    d.id = "KEN";
    d.district = "Kensington";
    d.title = "Kensington: Kensington Gore and Queen's Gate";
    d.lat0 = 51.4998;
    d.lon0 = -0.1788;
    d.source = "OpenStreetMap, Overpass extract of 2026-11-18T18:32:48Z (bbox 51.4955,-0.1845,51.5040,-0.1730)";
    d.licence = "ODbL 1.0, (c) OpenStreetMap contributors";
    // Queen's Gate is a dual carriageway: its two one-way carriageways (OSM ways 5079973 and 140394196
    // southbound, 207825581 northbound) run 11 to 14 m apart either side of a central reservation with
    // crossovers at the side streets. It is modelled as one street along their midline, from the Kensington
    // Road junction (node 12195919) at -82.1 degrees, with a 4.5 m reservation and a 7.5 m carriageway each side.
    const double qx = -104.5, qy = 184.8, ux = 0.13678, uy = -0.99060;
    auto onQG = [&](double s) { return std::make_pair(qx + ux * s, qy + uy * s); };
    const auto qpc = onQG(205.2), qgt = onQG(292.2), qep = onQG(392.24), qgs = onQG(455.0);
    d.junctions = {
        {"KRW", -185.0, 181.4, 6.0, -1.0, true, "patch edge on Kensington Road (way 220765538)"},
        {"QGN", -104.5, 184.8, 6.0, -1.0, false, "OSM node 12195919, Kensington Road x Queen's Gate"},
        {"KGW", 45.8, 198.1, 40.0, -1.0, false, "OSM node 20947178, a bend of Kensington Gore (the side road west of the Royal Albert Hall is not modelled)"},
        {"KGE", 190.0, 201.9, 6.0, -1.0, true, "patch edge on Kensington Gore (near node 2084200144)"},
        {"QPC", qpc.first, qpc.second, 5.0, -1.0, false, "midline of nodes 860183706 and 276549, Queen's Gate x Prince Consort Road"},
        {"QGT", qgt.first, qgt.second, 5.0, -1.0, false, "midline of nodes 860183709 and 276551, Queen's Gate x Queen's Gate Terrace"},
        {"QEP", qep.first, qep.second, 5.0, -1.0, false, "midline of nodes 276550 and 3960600156, Queen's Gate x Elvaston Place"},
        {"QGS", qgs.first, qgs.second, 6.0, -1.0, true, "patch edge on Queen's Gate"},
        {"PCE", 25.0, -6.06, 6.0, -1.0, true, "patch edge on Prince Consort Road (between nodes 27481062 and 3358814596)"},
        {"QTG", -150.2, -114.5, 4.0, -1.0, false, "OSM node 27481160, Queen's Gate Terrace x Gore Street"},
        {"QTW", -194.9, -119.68, 6.0, -1.0, true, "patch edge on Queen's Gate Terrace"},
        {"EGS", -141.0, -214.3, 4.0, -1.0, false, "OSM node 33709382, Elvaston Place x Gore Street"},
        {"EPW", -185.67, -219.79, 6.0, -1.0, true, "patch edge on Elvaston Place"},
    };
    const std::string main = "inferred: two traffic lanes each way of 3.2 m (OSM lanes=3 to 6, turning lanes included); the modelled lane is the nearside one";
    d.streets = {
        {"Kensington Road", "KRW", "QGN", 0, 6.4, 0.0, 4.75, "OSM ways 220765538, 220765536 (primary)", main},
        {"Kensington Gore", "QGN", "KGW", 0, 6.4, 0.0, 4.75, "OSM ways 72454100 (Kensington Road), 1418842639, 198261233 (primary)", main},
        {"Kensington Gore", "KGW", "KGE", 0, 6.4, 0.0, 4.75, "OSM ways 1418842641, 372971026, 1418842640, 813381224, 198261231 (primary)", main},
    };
    const std::string qg = "inferred from the carriageways' separation (11 to 14 m) and parking:both=lane: 7.5 m carriageway each way "
                           "(3.5 m lane, parking bays both sides), 4.5 m central reservation";
    const char* qgWays[] = {"OSM ways 5079973 (southbound), 207825581 (northbound)", "OSM ways 140394196 (southbound), 207825581 (northbound)",
                            "OSM ways 140394196 (southbound), 207825581 (northbound)", "OSM ways 2693195 (southbound), 1436007720 (northbound)"};
    const char* qgEnds[][2] = {{"QGN", "QPC"}, {"QPC", "QGT"}, {"QGT", "QEP"}, {"QEP", "QGS"}};
    for (int i = 0; i < 4; ++i) {
        d.streets.push_back({"Queen's Gate", qgEnds[i][0], qgEnds[i][1], 0, 9.75, 4.5, 6.0, qgWays[i], qg});
    }
    const std::string minor = "inferred: one 3.0 m lane each way; parking bays where OSM tags parking:both=lane";
    d.streets.push_back({"Prince Consort Road", "QPC", "PCE", 0, 4.8, 0.0, 1.8, "OSM ways 2693150, 943673804 (residential, lanes=2)", minor});
    d.streets.push_back({"Queen's Gate Terrace", "QGT", "QTG", 0, 5.2, 0.0, 1.6, "OSM ways 1459124443, 4825959 (parking:both=lane)", minor});
    d.streets.push_back({"Queen's Gate Terrace", "QTG", "QTW", 0, 5.2, 0.0, 1.6, "OSM way 4825959 (parking:both=lane)", minor});
    d.streets.push_back({"Gore Street", "QTG", "EGS", 0, 4.3, 0.0, 1.5, "OSM way 8419272 (residential)", minor});
    d.streets.push_back({"Elvaston Place", "QEP", "EGS", 0, 4.6, 0.0, 1.5, "OSM ways 1436007721, 876112133 (oneway=no)", minor});
    d.streets.push_back({"Elvaston Place", "EGS", "EPW", 0, 4.6, 0.0, 1.5,
                         "OSM ways 4825945, 876112129: two one-way carriageways 5 m apart, modelled as one two-way street", minor});
    // No turn restriction recorded in OpenStreetMap applies to these streets (the one at node 12195919 is for
    // the road out of Kensington Gardens, which is not modelled). U-turns are never built.
    return d;
}

DistrictSpec districtKnightsbridge() {
    DistrictSpec d;
    d.id = "KNB";
    d.district = "Knightsbridge";
    d.title = "Knightsbridge: Brompton Road, Hans Road and Basil Street";
    d.lat0 = 51.4998;
    d.lon0 = -0.1640;
    d.source = "OpenStreetMap, Overpass extract of 2026-11-18T18:32:48Z (bbox 51.4965,-0.1700,51.5032,-0.1585)";
    d.licence = "ODbL 1.0, (c) OpenStreetMap contributors";
    d.junctions = {
        {"BRW", -117.9, -138.7, 6.0, -1.0, true, "patch edge on Brompton Road (node 109677; Beaufort Gardens is not modelled)"},
        {"HRJ", -33.7, -68.8, 6.0, -1.0, false, "OSM node 109675, Brompton Road x Hans Road"},
        {"BRM", 47.5, 2.8, 40.0, -1.0, false, "OSM node 26389702, a bend of Brompton Road (Lancelot Place is not modelled)"},
        {"BR2", 91.6, 49.2, 40.0, -1.0, false, "OSM node 6983561823, a bend of Brompton Road"},
        {"BRG", 166.9, 149.8, 40.0, -1.0, false, "OSM node 1685938590, a bend of Brompton Road (Knightsbridge Green is not modelled)"},
        {"BRE", 189.0, 174.9, 6.0, -1.0, true, "patch edge on Brompton Road, before the Knightsbridge junction"},
        {"HR1", 40.3, -130.3, 20.0, -1.0, false, "OSM node 1233359613, a bend of Hans Road"},
        {"HR2", 69.1, -146.0, 15.0, 9.5, false, "OSM node 7972229618, a bend of Hans Road"},
        {"HBJ", 123.3, -156.3, 4.5, -1.0, false,
         "intersection of the Hans Road and Basil Street centrelines, 5.5 m east of OSM node 26389722 (Walton Place is not modelled)"},
        {"HRS", 158.4, -166.3, 6.0, -1.0, true, "patch edge on Hans Road towards Hans Place"},
        {"HCB", 171.6, -56.0, 5.0, -1.0, false,
         "OSM node 26389720, Basil Street x Hans Crescent (the 12 m of Hans Crescent to the north-west and Basil Street beyond are not modelled)"},
        {"HCE", 205.9, -81.3, 6.0, -1.0, true, "patch edge on Hans Crescent (node 26389719)"},
    };
    // Widths: the mapped centrelines run off-centre between the building lines (on Brompton Road 13 to 17 m
    // from the north-west frontage, 7 to 10 m from Harrods'), so each side's kerb is set from that side's
    // building line, leaving at least 2 m of pavement; one traffic lane each way as OSM records (lanes=2).
    const std::string br = "inferred: OSM lanes=2 (one each way); kerbs 6.0 m north-west and 4.4 m south-east of the mapped centreline, "
                           "from the building lines either side (Harrods' frontage 6.8 m from it)";
    const std::string one = "inferred: a one-way residential street, kerbs set from the building lines either side, the lane on its left";
    const std::string res = "inferred: one 3.0 m lane each way with room to the kerb";
    auto asym = [](DistrictStreet s, double right) {
        s.halfWidthRight = right;
        return s;
    };
    d.streets = {
        {"Brompton Road", "BRW", "HRJ", 0, 6.0, 0.0, 2.2, "OSM ways 498269362, 1070232814, 1070232813 (trunk, lanes=2)",
         "inferred: OSM lanes=2 (one each way), 6.0 m from the centreline to each kerb"},
        asym({"Brompton Road", "HRJ", "BRM", 0, 6.0, 0.0, 2.2, "OSM ways 1070232813, 5079978 (trunk, lanes=2)", br}, 4.4),
        asym({"Brompton Road", "BRM", "BR2", 0, 6.0, 0.0, 2.2, "OSM way 5079978 (trunk, lanes=2)", br}, 4.4),
        asym({"Brompton Road", "BR2", "BRG", 0, 6.0, 0.0, 2.2, "OSM way 5079978 (trunk, lanes=2)", br}, 4.4),
        asym({"Brompton Road", "BRG", "BRE", 0, 6.0, 0.0, 2.2, "OSM way 5079978 (trunk, lanes=2)", br}, 4.4),
        asym({"Hans Road", "HRJ", "HR1", 1, 3.3, 0.0, 1.6, "OSM way 4398741 (oneway=yes)", one}, 4.2),
        {"Hans Road", "HR1", "HR2", 1, 4.2, 0.0, 1.6, "OSM way 4398741 (oneway=yes)", one},
        asym({"Hans Road", "HR2", "HBJ", 1, 4.4, 0.0, 1.6, "OSM way 4398741 (oneway=yes)", one}, 2.6),
        {"Hans Road", "HBJ", "HRS", 0, 3.8, 0.0, 1.6, "OSM way 226148622 (oneway=no)", res},
        asym({"Basil Street", "HBJ", "HCB", 1, 4.6, 0.0, 1.0, "OSM way 10809271 (oneway=yes)", one}, 2.2),
        {"Hans Crescent", "HCB", "HCE", 0, 4.4, 0.0, 1.6, "OSM ways 24461996, 858630639 (residential)", res},
    };
    d.banned = {{"HBJ", "HRS>HBJ", "HBJ>HCB"}};
    d.bannedSource = {"OSM relation 9932431: no_right_turn from Hans Road (way 226148622) via node 26389722 to Basil Street (way 10809271)"};
    return d;
}

}  // namespace road

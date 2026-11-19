#pragma once
// District networks: compact, bounded reconstructions of real streets (two London patches, left-hand traffic;
// one San Francisco patch, right-hand traffic), built by the same rules as the synthetic grids (lane segments
// and turning movements as graph nodes, no U-turns, banned turns never built) but from street centrelines in a
// local metric frame. A district sets its traffic side: the modelled lane lies on that side of the centreline,
// and the turn across the opposing lane costs the larger penalty.
//
// They are source-informed research reconstructions with approximated details, not surveyed digital twins.
// Sourced: junction positions, street names, the order and adjacency of streets, one-way rules and turn
// restrictions, from OpenStreetMap (ODbL 1.0, (c) OpenStreetMap contributors). Inferred: carriageway widths,
// lane offsets, kerb radii, central reservations, where lanes end at a junction and the turning paths.
// Simplified: centrelines are straight between junctions and bend points (within a few metres of the
// mapped ones; road_driving/viewer/districts/PROVENANCE.md lists the largest deviation per street), minor
// side streets are left out (their kerb is drawn continuous), and a dual carriageway is modelled as one
// street with a central reservation about its midline.
//
// Patch edges ("open" junctions) close the modelled carriageway where a street carries on beyond the patch:
// a boundary line bounds the road for clearance but is no kerb, so the kerb scanner sees nothing there.
#include "road_driving/lane_network.hpp"

#include <string>
#include <utility>
#include <vector>

namespace road {

struct DistrictJunction {
    std::string name;
    double x = 0.0, y = 0.0;
    double kerbRadius = 6.0;  // corner kerb radius; at a bend (two streets), the inner kerb's radius
    double mouth = -1.0;      // > 0: lanes end and start this far from the junction (default: from its corners)
    bool open = false;        // a patch edge: the street carries on beyond the modelled area
    std::string source;       // where the position comes from, e.g. "OSM node 12195919"
};

struct DistrictStreet {
    std::string name;         // the real street's name
    std::string a, b;         // junction names
    int oneWay = 0;           // 0: two-way; +1: a -> b only; -1: b -> a only
    double halfWidth = 4.0;   // centreline to the outer kerb on the left of a -> b, m
    double median = 0.0;      // width of a central reservation, m (0: none)
    double laneOffset = 1.5;  // the modelled lane's centre from the centreline, m: on the traffic side of the direction of travel
    std::string source;       // the OSM ways the centreline follows
    std::string widthBasis;   // how the width was inferred
    double halfWidthRight = -1.0;  // centreline to the kerb on the right of a -> b (< 0: as on the left)
    double right() const { return halfWidthRight < 0.0 ? halfWidth : halfWidthRight; }
};

struct DistrictSpec {
    std::string id;           // the network name, e.g. "KEN"
    std::string title;        // e.g. "Kensington: Queen's Gate and Kensington Gore"
    std::string district;     // "Kensington", "Knightsbridge", "SoMa"
    std::string city;         // "San Francisco" (empty for the London districts, which predate the field)
    double lat0 = 0.0, lon0 = 0.0;  // origin of the local frame: x east, y north, metres
    // How latitude and longitude become the local frame. Empty: the London districts' equirectangular scaling
    // (111320 cos(lat0) and 110574 m per degree, road_driving/tools/district_extract.py); otherwise the text
    // here, e.g. the WGS84 tangent plane at the origin.
    std::string frame;
    std::string source;       // the extract the geometry was taken from
    std::string licence;
    bool rightHand = false;   // traffic side (NetworkSpec::rightHand)
    std::vector<DistrictJunction> junctions;
    std::vector<DistrictStreet> streets;
    std::vector<BannedTurn> banned;          // junction, from lane, to lane
    std::vector<std::string> bannedSource;   // parallel to banned
    // Preferred lane-path radii, m. In left-hand traffic a left turn keeps to the near side and a right turn
    // crosses the opposing lane on a wider path; in right-hand traffic the reverse, so a district sets them to
    // match its traffic side.
    double leftRadius = 9.0, rightRadius = 14.0, bendRadius = 60.0;
    double farRadius() const { return rightHand ? leftRadius : rightRadius; }  // the turn across the opposing lane
};

DistrictSpec districtKensington();      // "KEN"
DistrictSpec districtKnightsbridge();   // "KNB"
DistrictSpec districtSanFranciscoSoma();  // "SF-SOMA": right-hand traffic
std::vector<std::string> districtNames();
bool findDistrict(const std::string& name, DistrictSpec& out);

// The lane network of a district. Throws std::invalid_argument when the geometry does not fit (a street too
// short for its junctions, a kerb that does not close); validate() reports what remains.
LaneNetwork buildDistrict(const DistrictSpec& d, const NetworkSpec& spec = NetworkSpec{});

// Signed distance to the nearest kerb or patch edge (positive on the carriageway) and the unit normal there,
// pointing into the carriageway. The sign comes from the nearest kerb point: its normal, or at a join of two
// kerb pieces the mean of theirs, which is exact for closed kerb loops.
double districtKerbDistance(const LaneNetwork& net, double x, double y, double& nx, double& ny);
std::vector<std::string> validateDistrict(const LaneNetwork& net);

// The carriageway's boundary as closed loops with the carriageway on their left (anticlockwise outer loops,
// clockwise islands), arcs sampled at most `step` radians apart; and an even-odd test against them.
std::vector<std::vector<std::pair<double, double>>> carriagewayLoops(const LaneNetwork& net, double step = 0.02);
bool insideCarriageway(const std::vector<std::vector<std::pair<double, double>>>& loops, double x, double y);

}  // namespace road

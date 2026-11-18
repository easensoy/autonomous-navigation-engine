# District scenes: provenance

Two compact, bounded scenes of real London streets, each with its own network in the backend (`KEN` and `KNB`, [`src/district.cpp`](../../src/district.cpp)) and its own scenery file here. They are **source-informed research reconstructions with approximated details, not surveyed digital twins**. They are two separate patches, about 1.5 km apart, shown as separate chapters: no continuous route joins them.

| | Kensington (`KEN`) | Knightsbridge (`KNB`) |
|---|---|---|
| Streets | Kensington Road and Kensington Gore, Queen's Gate (dual carriageway), Prince Consort Road, Queen's Gate Terrace, Gore Street, Elvaston Place | Brompton Road, Hans Road, Basil Street, Hans Crescent |
| Junctions modelled | 7, plus 6 patch edges | 8 (including 5 bends), plus 4 patch edges |
| Route | `KEN-A`: Kensington Gore westbound past the Royal Albert Hall, left into Queen's Gate, the length of Queen's Gate southbound, right into Elvaston Place (696 m) | `KNB-A`: Brompton Road south-westbound past Harrods, left into Hans Road, left into Basil Street, right into Hans Crescent (637 m) |
| Local frame | x east, y north, metres from 51.4998 N, 0.1788 W | x east, y north, metres from 51.4998 N, 0.1640 W |
| Scenery file | [`kensington.json`](kensington.json): 408 building outlines, 17 parks and lawns, 73 trees, 108 walls, fences and railings | [`knightsbridge.json`](knightsbridge.json): 624 building outlines, 4 parks and gardens, 26 trees, 73 walls, fences and railings |

## Sources

- **OpenStreetMap**, through the Overpass API, extracted at the database state of **2026-11-18T18:32:48Z**. The bounding boxes were 51.4955, -0.1845, 51.5040, -0.1730 (Kensington) and 51.4965, -0.1700, 51.5032, -0.1585 (Knightsbridge). `python road_driving/tools/district_extract.py --query KEN` prints the exact query. The raw extracts are attached to the release that carries the showcase video, so the scenery files can be regenerated from them.
- **Historic England, National Heritage List for England**, for the two landmarks and the terrace type. Their list-entry numbers are carried in OpenStreetMap's `heritage` tags:
  - [Royal Albert Hall](https://historicengland.org.uk/listing/the-list/list-entry/1217742) (grade I): red brick and buff terracotta, an elliptical plan, two main tiers of windows (square-headed below, round-headed above), a frieze of figures, and a glazed dome.
  - [Harrods](https://historicengland.org.uk/listing/the-list/list-entry/1294346) (grade II*):
    - the Brompton Road frontage of 1901-05 by C. W. Stephens, in pinky-buff Doulton terracotta;
    - two lower storeys of large display windows and four storeys above;
    - giant pilasters;
    - octagonal corner cupolas and mansard attics in copper;
    - an octagonal dome with terracotta ribs and facings, and a cupola above.
  - [68-87 Queen's Gate](https://historicengland.org.uk/listing/the-list/list-entry/1265993) (grade II, about 1856-9):
    - stucco, four storeys with an attic and basement, three windows wide;
    - channelled rustication to the ground floor and Doric porches;
    - a balustraded first-floor balcony, and corniced first- and second-floor windows;
    - a main cornice above the third floor, and area railings.
- **The Royal Borough of Kensington and Chelsea**, which publishes the [Queen's Gate Conservation Area Appraisal](https://www.rbkc.gov.uk/media/document/queens-gate-conservation-area-appraisal) (adopted 26 February 2020) as its account of that area's character. It is cited as the borough's reference; this reconstruction reproduces nothing from it.

## Sourced, inferred and decorative

**Sourced, from OpenStreetMap, as mapped:**
- the position of every junction and bend (an OSM node, or on Queen's Gate the midpoint of its two carriageways' nodes);
- street names, their order and their adjacency;
- one-way rules (Hans Road from Brompton Road, Basil Street);
- the turn restriction modelled: no right turn from Hans Road (from Hans Place) into Basil Street, relation 9932431;
- building outlines, storey counts (`building:levels`), names, types and heritage status;
- parks, lawns and gardens, trees, walls, fences and railings;
- the mapped centrelines, drawn by the provenance overlay (key `V`).

**Inferred, in the backend:**
- carriageway widths: from the OSM `lanes` and `parking` tags, and in Knightsbridge from the building lines either side (below);
- the modelled lane and its offset;
- Queen's Gate's 4.5 m central reservation;
- corner kerb radii;
- where lanes end at each junction;
- the turning paths, which are clothoid-arc-clothoid curves on the lanes' intersection, with their curvature limit and the vehicle's swept clearance checked.

**Inferred, in the viewer:**
- building heights: 3.2 m a storey where OSM gives storeys and no height; a style's default where it gives neither;
- each building's style, from its tags, name and the street it fronts.

**Decorative and approximate, in the viewer only:**
- facades, materials and roofs;
- the trees on Queen's Gate's reservation and the extra park trees;
- lamp columns, bollards, one telephone kiosk and one pillar box;
- street name plates;
- road markings: centre lines, double yellow lines, Queen's Gate's parking bays;
- shop fascias with generic names.

No trademark, lettering, emblem or relief from a real building is reproduced.

The simulation uses only the backend's network: its kerb lines and arcs, which bound the carriageway and are what the kerb scanner sees, and its lanes. The viewer draws that same kerb from the backend's map message. No building or tree stands on the carriageway or within 0.5 m of a kerb: the viewer's tests check every building outline and every tree. Lamp columns and bollards are placed 0.5 m or more back from the kerb. Buildings stand 2.6 m or more from Kensington's modelled kerbs, and 1.66 m or more from Knightsbridge's.

## How far the model departs from the map

Centrelines are straight between junctions and bends. The largest distance of a mapped OSM node from the modelled centreline along each street is measured by `python road_driving/tools/district_deviation.py KEN <extract>`:

| Street | Modelled | OSM ways | Largest distance |
|---|---|---|---|
| Kensington Road | KRW-QGN | 220765538, 220765536 | 1.9 m |
| Kensington Gore | QGN-KGW | 72454100, 1418842639, 198261233 | 2.0 m |
| Kensington Gore | KGW-KGE | 1418842641, 372971026, 1418842640, 813381224, 198261231 | 2.4 m |
| Queen's Gate | QGN-QPC | 5079973, 207825581 | its two carriageways run 5.6 to 6.9 m either side of the modelled midline |
| Queen's Gate | QPC-QGT | 140394196, 207825581 | (mapped nodes only at its junctions) |
| Queen's Gate | QGT-QEP | 140394196, 207825581 | 6.1 m either side of the midline |
| Queen's Gate | QEP-QGS | 2693195, 1436007720 | 6.3 to 7.5 m either side of the midline |
| Prince Consort Road | QPC-PCE | 2693150, 943673804 | 0.2 m |
| Queen's Gate Terrace | QGT-QTG, QTG-QTW | 1459124443, 4825959 | 0.1 m |
| Gore Street | QTG-EGS | 8419272 | 1.1 m |
| Elvaston Place | QEP-EGS | 1436007721, 876112133 | 0.7 m |
| Brompton Road | BRW-HRJ to BRG-BRE (five pieces) | 498269362, 1070232814, 1070232813, 5079978 | 0.0 to 1.1 m |
| Hans Road | HRJ-HR1, HR1-HR2, HR2-HBJ | 4398741 | 0.3, 1.5, 1.5 m |
| Hans Road | HBJ-HRS | 226148622 | 0.1 m |
| Basil Street | HBJ-HCB | 10809271 | 0.9 m |
| Hans Crescent | HCB-HCE | 24461996, 858630639 | 2.2 m |

Knightsbridge's mapped centrelines run off-centre between the building lines. On Brompton Road they run 13 to 17 m from the north-west frontage and 7 to 10 m from Harrods. Each side's kerb is therefore set from that side's building line, leaving at least 2 m of pavement: Brompton Road's kerbs are 6.0 m north-west and 4.4 m south-east of the mapped centreline.

## What is left out or simplified

- **Queen's Gate** is a dual carriageway of two one-way ways with crossovers at the side streets. It is modelled as one two-way street along their midline, with a 4.5 m reservation and a crossover at each modelled junction.
- **Side streets that are not modelled** have their kerb drawn continuous, as the backend has it: Jay Mews, the roads either side of the Royal Albert Hall, Bremner Road, Imperial College Road, Queen's Gate Mews, Hyde Park Gate, the park road from Kensington Gardens, Lancelot Place, Knightsbridge Green, Beaufort Gardens, Montpelier Street, Brompton Place and Walton Place.
- **Elvaston Place west of Gore Street** is mapped as two one-way ways 5 m apart. It is modelled as one two-way street.
- **The north-west end of Hans Crescent** is pedestrian, and is not modelled. The 12 m of it beyond Basil Street, and the two-way part of Basil Street further north, are also left out.
- **Patch edges** are where a street carries on beyond the modelled area. A boundary line closes the carriageway there for the clearance checks, but it is no kerb and the kerb scanner sees nothing on it. The viewer draws a hatched apron, level with the road.
- **Exact positions are not claimed.** Kerb positions, carriageway widths, parking, bus lanes, crossings, signals and the junctions' detailed geometry are approximations. Building heights come from storey counts, and the facades show each type's elements and proportions, not a survey of any building.

## Licence

The scenery files [`kensington.json`](kensington.json) and [`knightsbridge.json`](knightsbridge.json), and the junction coordinates in `src/district.cpp`, are derived from OpenStreetMap data, © OpenStreetMap contributors. They are available under the [Open Database License 1.0](https://opendatacommons.org/licenses/odbl/1-0/). Anything produced from them, such as the rendered scenes, screenshots and video, must credit "© OpenStreetMap contributors". The facts quoted from Historic England's list entries are from text available under the Open Government Licence v3.0.

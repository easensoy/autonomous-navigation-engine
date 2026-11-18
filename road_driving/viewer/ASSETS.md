# Asset provenance

No third-party 3D model, texture, logo or photograph is included. The vehicle carries no manufacturer's or operator's badge. The district scenes use OpenStreetMap data (ODbL, below); every model, texture and facade in them is original.

| Asset | Where | Source | Licence and redistribution |
|---|---|---|---|
| Vehicle proxy: body, glazing, lights, mirrors, wheels, roof rig, kerb scanner | `ane_viewer/vehicle_mesh.py` | original, generated procedurally for this repository | the repository's; redistributable |
| Street scene: carriageway, kerbs, markings, buildings, street furniture | `ane_viewer/world.py`, `ane_viewer/geometry.py` | generated at run time from the simulation's map; buildings and furniture procedural | the repository's; redistributable |
| District scenery data: building outlines, storeys, names, parks, trees, walls and railings, mapped centrelines | `districts/kensington.json`, `districts/knightsbridge.json` | derived from OpenStreetMap by `road_driving/tools/district_extract.py` ([PROVENANCE.md](districts/PROVENANCE.md)) | [ODbL 1.0](https://opendatacommons.org/licenses/odbl/1-0/), © OpenStreetMap contributors; redistributable under the ODbL, with attribution |
| District junction positions | `road_driving/src/district.cpp` | derived from OpenStreetMap nodes | ODbL 1.0, © OpenStreetMap contributors |
| District facades, roofs, landmarks, trees, lamps, markings | `ane_viewer/facades.py`, `ane_viewer/district_scene.py` | original geometry, laid out after Historic England list entries for the building types ([PROVENANCE.md](districts/PROVENANCE.md)); no lettering, emblem or relief of a real building | the repository's; redistributable |
| Textures: brick, stucco, Portland stone, terracotta, slate, paving, asphalt, glazing, grass, railings, hatching | `ane_viewer/materials.py` | original, generated procedurally at start from fixed seeds; no image file | the repository's; redistributable |
| The showcase video and its poster | GitHub release `viewer-london-v1`; `road_driving/docs/media/` | captured from this viewer | the repository's; the district chapters show OpenStreetMap-derived scenes: credit "© OpenStreetMap contributors" |
| Screenshots and video in `road_driving/docs/media/` | | captured from this viewer | the repository's; redistributable |
| Text font | Segoe UI where Windows provides it, DejaVu Sans on Linux, else Panda3D's built-in default font | the operating system, or Panda3D | used where installed; not redistributed here |
| Panda3D 1.10.16 | `requirements.txt` | PyPI | Modified BSD; installed by the user, not redistributed |
| panda3d-gltf 1.3.0, and panda3d-simplepbr, which it installs | `requirements.txt` | PyPI | BSD-3-Clause; installed by the user, not redistributed |
| ffmpeg, optional, for encoding recordings | found at run time (`ANE_FFMPEG`, PATH, imageio-ffmpeg) | | LGPL or GPL builds; not redistributed |

## Why a proxy, not an I-PACE model

On 18 November 2026 a search found no Jaguar I-PACE model that could be both obtained without purchase or an account and redistributed:
- [CGTrader](https://www.cgtrader.com/3d-models/car/suv/jaguar-i-pace-2024), RenderHub and TurboSquid sell theirs.
- One [TurboSquid model](https://www.turbosquid.com/3d-models/3d-i-pace-pace-2019-model-1315241) is for editorial use only.
- [3D CAD Browser](https://www.3dcadbrowser.com/3d-model/jaguar-i-pace-2019) offers free models but forbids redistributing their source files, and none of them is glTF.
- The [Free3D](https://free3d.com/3d-model/2016-jaguar-i-pace-8646.html) model states no clear licence and is not glTF.

The viewer therefore ships an original, I-PACE-inspired proxy. [`models/README.md`](models/README.md) explains how to draw instead a model you have the right to use, with no change to the simulation.

## Reference geometry

The proxy's footprint is the simulation's own vehicle (`VehicleParams` in `include/road_driving/vehicle.hpp`):
- wheelbase 2.990 m;
- length 4.682 m;
- body width 2.011 m, and 2.139 m over the mirrors;
- overhangs 0.850 m at the front and 0.842 m at the rear.

These are published-figure approximations, as that header notes.

The drawing adds three more approximations: a height of 1.565 m, a wheel radius of 0.377 m (245/50 R20) and a track of 1.64 m.

The profile is a hand-made approximation of the I-PACE's general proportions: a short bonnet, a cab-forward windscreen, a long roof, and a tall tail with a lip spoiler. It is not traced from any drawing, photograph or model. `tests/test_vehicle.py` and `tests/test_panda.py` check the proxy against the footprint.

## The sensor rig

The roof dome, its camera band and the four fender and corner pods are decorative. They are generic shapes in the style of public robotaxi imagery, and correspond to no simulated sensor.

The simulated sensors, and how the viewer shows them:

| Sensor | Position (body frame) | Drawn as |
|---|---|---|
| Kerb scanner | front bumper centre, 3.84 m ahead of the rear axle, at kerb height; 37 beams over ±90°, 0.5-25 m, 10 Hz | a dark box with an orange stripe; each scan's beams and returns |
| GNSS antenna | 1.60 m ahead, on the roof | a dark puck; the raw fixes |
| IMU | 1.40 m ahead | not drawn (inside the car) |
| Wheel speed, steering angle | | not drawn |

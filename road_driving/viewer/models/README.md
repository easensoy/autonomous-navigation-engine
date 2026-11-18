# Drawing an imported vehicle model

By default the viewer draws its own procedural proxy (`ane_viewer/vehicle_mesh.py`). An imported glTF or GLB model can replace it, with no change to the simulation: the model is only placed at the pose the backend reports.

1. Put the model file here (`*.glb` and `*.gltf` in this folder are not committed).
2. Write a JSON file next to it, for example `my_vehicle.json`:

```json
{
  "model": "my_vehicle.glb",
  "forward": "-Z",
  "up": "+Y",
  "fit_length": true,
  "add_rig": true,
  "add_wheels": false,
  "wheels": {"fl": "WheelFrontLeft", "fr": "WheelFrontRight", "rl": "WheelRearLeft", "rr": "WheelRearRight"},
  "source": "where it came from (URL)",
  "licence": "its licence",
  "attribution": "the attribution its licence requires"
}
```

3. Start the viewer with `--vehicle-model road_driving/viewer/models/my_vehicle.json`.

**Fields:**
- **Axes.** `forward` and `up` are the model's own axes. glTF models usually face -Z or +Z with +Y up.
- **Scale.** With `fit_length` the model is scaled to the simulated vehicle's length (4.682 m). Otherwise `scale` applies.
- **Placement.** The rear bumper goes 0.842 m behind the rear axle, the model is centred across, and its lowest point sits on the ground (`ground_offset` adjusts this).
- **Wheels.** `wheels` names the model's wheel nodes, if it has any, so they steer and spin. Otherwise `add_wheels` draws the proxy's wheels.
- **Sensors.** `add_rig` adds the decorative roof rig. The active kerb scanner is always drawn at its simulated mount, 3.84 m ahead of the rear axle.

**Alignment report.** On start the viewer prints how the placed model compares with the simulated footprint (length 4.682 m, body width 2.011 m, 2.139 m over the mirrors). Check this report before relying on the drawing.

**Licence.** Use only a model whose licence allows your use. Record its source, licence and attribution in the JSON file and in `../ASSETS.md` if you publish anything made with it.

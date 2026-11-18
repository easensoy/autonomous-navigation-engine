"""Placing an imported vehicle model in the simulation's body frame. Pure arithmetic (no Panda3D)."""


def alignment(bmin, bmax, params, cfg):
    """Scale and offset that put a model with these bounds (already in body axes) into the body frame.

    Pure arithmetic (testable without Panda3D). Returns (scale, offset, report)."""
    length = bmax[0] - bmin[0]
    width = bmax[1] - bmin[1]
    if cfg.get("fit_length", True):
        scale = params["length"] / length if length > 0 else 1.0
    else:
        scale = float(cfg.get("scale", 1.0))
    # Rear bumper at -rear_overhang, centred across, wheels on the ground.
    offset = (-params["rear_overhang"] - scale * bmin[0], -scale * 0.5 * (bmin[1] + bmax[1]), -scale * bmin[2] + float(cfg.get("ground_offset", 0.0)))
    report = {
        "model_length_m": round(scale * length, 3), "footprint_length_m": params["length"],
        "model_width_m": round(scale * width, 3), "footprint_width_m": params["width"], "footprint_width_mirrors_m": params["width_mirrors"],
        "length_error_m": round(scale * length - params["length"], 3), "width_error_m": round(scale * width - params["width"], 3),
        "scale": round(scale, 6),
    }
    return scale, offset, report

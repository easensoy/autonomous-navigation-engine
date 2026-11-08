"""Hero figure: the navigator's trip, the blockage and the replan, from docs/data/hero.json.

Positions come from the exported timeline (one entry per NavigationCore::executeNavigationStep);
routes are the navigator's own getCurrentPath() before and after handleObstacles. Playback time is
not computation time: the measured plan/replan times come from docs/data/summary.json.
"""

from __future__ import annotations

import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import FancyBboxPatch, Rectangle
from PIL import Image

from common import ASSETS, DATA, MONO, THEMES, Scene, apply_rc, draw_endpoints, draw_floor, load_json

FPS = 20
WIDTH_IN, HEIGHT_IN = 9.6, 6.6


class Story:
    """Everything the hero shows, read from the exported files."""

    def __init__(self):
        self.hero = load_json("hero.json")
        self.summary = load_json("summary.json")["hero"]
        self.search = load_json("search.json")
        self.scene = Scene(self.hero)
        h = self.hero
        self.start, self.goal = h["start"], h["goal"]
        self.blocked = h["blocked_nodes"]
        self.initial = h["routes"][0]["route"]["nodes"]
        self.final = h["routes"][-1]["route"]["nodes"]
        self.block_index = h["replan"]["route_index"]
        self.block_step = h["replan"]["step"]
        self.position = h["replan"]["position"]
        self.abandoned = h["replan"]["abandoned_remainder"]["nodes"]
        self.new_remainder = h["replan"]["new_remainder"]["nodes"]
        self.timeline = h["timeline"]  # [step, node, route_version, remaining_cost]
        self.steps = self.timeline[-1][0]
        self.reference = self.search["reference_shortest_length"]
        # The exported timeline must agree with the routes it came from.
        for step, node, version, _ in self.timeline:
            route = self.initial if version == 0 else self.final
            assert route[step] == node, (step, node)

    def cost_so_far(self, step: int) -> float:
        nodes = [self.timeline[k][1] for k in range(step + 1)]
        return self.scene.route_cost(nodes)

    def remaining(self, step: int) -> float:
        return self.timeline[step][3]


def ms(x: float) -> str:
    return f"{x:.2f} ms"


class HeroFrame:
    """One matplotlib figure, updated in place for every animation frame."""

    def __init__(self, story: Story, theme_name: str, *, dpi=100, static=False):
        self.s = story
        self.t = THEMES[theme_name]
        self.static = static
        apply_rc(self.t)
        self.fig = plt.figure(figsize=(WIDTH_IN, HEIGHT_IN), dpi=dpi)
        t = self.t
        sc = story.scene
        tm = story.summary["timing_ms"]

        self.fig.text(0.03, 0.958, "Replanning around a blocked aisle", fontsize=15.5, fontweight="bold",
                      color=t.text, va="center")
        self.status = self.fig.text(0.03, 0.915, "", fontsize=10.5, color=t.muted, va="center")
        self.counter = self.fig.text(0.97, 0.958, "", fontsize=11, color=t.muted, va="center", ha="right",
                                     family=MONO)

        self.ax = self.fig.add_axes([0.0, 0.255, 1.0, 0.635])
        ax = self.ax
        draw_floor(ax, sc, t, dot_size=2.0)
        ax.set_xlim(-4.6, 47.4)
        ax.set_ylim(-0.7, 23.7)
        for name, y in (("aisle A", 6.5), ("aisle B", 11.5), ("aisle C", 16.5)):
            ax.text(-0.85, y, name, fontsize=9.5, color=t.muted, ha="right", va="center")
        zx0, zy0, zx1, zy1, mult, _ = sc.zones[0]
        ax.text((zx0 + zx1) / 2, zy1 + 0.35, f"picking zone: cost ×{mult:g} per unit length", fontsize=9.5,
                color=t.text, ha="center", va="bottom", zorder=12,
                bbox=dict(boxstyle="round,pad=0.25", facecolor=t.background, edgecolor="none", alpha=0.85))
        draw_endpoints(ax, sc, t, story.start, story.goal, start_label="dock")
        gx, gy = sc.xy[story.goal]
        ax.annotate("station", (gx, gy), xytext=(0, 12), textcoords="offset points", ha="center", va="bottom",
                    fontsize=10, color=t.text, zorder=10)

        # Dynamic artists.
        self.reference_line, = ax.plot([], [], color=t.zone, lw=2.0, ls=(0, (1.2, 1.6)), solid_capstyle="round",
                                       zorder=4, alpha=0.0)
        self.previous_line, = ax.plot([], [], color=t.previous, lw=2.0, ls=(0, (4, 3)), zorder=4.5)
        self.planned_line, = ax.plot([], [], color=t.route_light, lw=2.6, solid_capstyle="round",
                                     solid_joinstyle="round", zorder=5)
        self.traversed_line, = ax.plot([], [], color=t.route, lw=4.0, solid_capstyle="round",
                                       solid_joinstyle="round", zorder=6)
        mid = story.reference["nodes"][len(story.reference["nodes"]) * 3 // 4]
        self.reference_leader, = ax.plot([sc.xy[mid][0]] * 2, [21.1, sc.xy[mid][1] + 0.25], color=t.zone, lw=1.0,
                                         alpha=0.0, zorder=11)
        self.reference_label = ax.text(sc.xy[mid][0], 21.75,
                                       f"shortest by distance: {story.reference['length']:.1f} u, "
                                       f"cost {story.reference['cost']:.1f}",
                                       fontsize=9.5, color=t.text, ha="center", va="center", zorder=12, alpha=0.0,
                                       bbox=dict(boxstyle="round,pad=0.25", facecolor=t.background, edgecolor=t.zone,
                                                 linewidth=1.0, alpha=0.0))
        bx0, by0, bx1, by1, _ = sc.blocks[0]
        self.block_patch = Rectangle((bx0, by0), bx1 - bx0, by1 - by0, facecolor=t.blocked, edgecolor=t.blocked,
                                     linewidth=1.6, alpha=0.0, zorder=7)
        ax.add_patch(self.block_patch)
        bxs = [sc.xy[n][0] for n in story.blocked]
        bys = [sc.xy[n][1] for n in story.blocked]
        self.block_marks = ax.scatter(bxs, bys, marker="x", s=26, color=t.blocked, linewidths=1.5, zorder=7.1,
                                      alpha=0.0)
        self.block_label = ax.text((bx0 + bx1) / 2, by1 + 0.35, f"blocked: {len(story.blocked)} nodes",
                                   fontsize=9.5, color=t.blocked, ha="center", va="bottom", fontweight="bold",
                                   zorder=12, alpha=0.0)
        self.callouts = []
        self.vehicle_ring = ax.scatter([], [], s=420, facecolors="none", edgecolors=t.blocked, linewidths=2.0,
                                       zorder=8, alpha=0.0)
        self.vehicle = ax.scatter([], [], s=95, color=t.marker, edgecolors=t.marker_ring, linewidths=1.8, zorder=9)

        self._legend()
        self._timeline(tm)
        self.fig.text(0.035, 0.018, "Playback: 1 frame step = 1 simulated navigation step (one graph node); "
                                    "not a vehicle model. Costs are edge weights; distances in coordinate units (u).",
                      fontsize=8.5, color=t.faint, va="bottom")

    def _legend(self):
        t = self.t
        handles = [
            Line2D([], [], color=t.route, lw=4.0, label="travelled"),
            Line2D([], [], color=t.route_light, lw=2.6, label="planned"),
            Line2D([], [], color=t.previous, lw=2.0, ls=(0, (4, 3)), label="earlier plan"),
            Line2D([], [], color=t.blocked, marker="x", lw=0, markersize=7, markeredgewidth=1.6,
                   label="reported blocked"),
            Line2D([], [], color=t.marker, marker="o", lw=0, markersize=8, markeredgecolor=t.marker_ring,
                   label="navigator"),
            Rectangle((0, 0), 1, 1, facecolor=t.zone, alpha=0.35, edgecolor=t.zone, hatch="////",
                      label="picking zone"),
        ]
        if self.static:
            handles.insert(3, Line2D([], [], color=t.zone, lw=2.0, ls=(0, (1.2, 1.6)),
                                     label="shortest by distance"))
        self.fig.legend(handles=handles, loc="center", bbox_to_anchor=(0.5, 0.215), ncol=4,
                        frameon=False, fontsize=9.5, handlelength=2.2, columnspacing=1.8, handletextpad=0.6,
                        labelcolor=t.text)

    def _timeline(self, tm):
        t = self.t
        s = self.s
        ax = self.fig.add_axes([0.06, 0.085, 0.58, 0.05])
        self.tl = ax
        ax.set_xlim(-0.8, s.steps + 0.8)
        ax.set_ylim(-1, 1)
        ax.axis("off")
        ax.plot([0, s.steps], [0, 0], color=t.rule, lw=6, solid_capstyle="round", zorder=1)
        self.tl_fill, = ax.plot([0, 0], [0, 0], color=t.route, lw=6, solid_capstyle="round", zorder=2)
        for step, label, color, ha in ((0, "plan", t.muted, "center"),
                                       (s.block_step, "blocked → replan", t.blocked, "center"),
                                       (s.steps, "arrive", t.muted, "center")):
            ax.plot([step], [0], marker="o", ms=7, color=color, markeredgecolor=t.background, zorder=3)
            ax.text(step, 0.95, label, fontsize=9, color=color, ha=ha, va="bottom")
            ax.text(step, -1.0, f"step {step}" if step == 0 else str(step), fontsize=8.5, color=t.faint,
                    ha="center", va="top", family=MONO)
        self.tl_head, = ax.plot([0], [0], marker="o", ms=9, color=t.marker, markeredgecolor=t.marker_ring,
                                markeredgewidth=1.5, zorder=4)
        self.cost_text = self.fig.text(0.68, 0.112, "", fontsize=10, color=t.text, va="center", family=MONO)
        self.fig.text(0.68, 0.066, f"measured: plan {ms(tm['plan']['p50'])}, replan {ms(tm['replan']['p50'])}\n"
                                   f"(median of {s.summary['timing_runs']:,} runs, not playback time)",
                      fontsize=8.5, color=t.faint, va="center", linespacing=1.35)

    # ------------------------------------------------------------------------------------------
    def set_state(self, *, step: int, phase: str, block_alpha=0.0, draw_fraction=1.0, reference_alpha=0.0,
                  pulse=0.0, reset=0.0):
        """reset in (0, 1]: fade from the arrival state back to the opening frame, so the loop is smooth."""
        s, sc, t = self.s, self.s.scene, self.t
        if reset > 0:
            self._set_reset(reset)
            return
        version = 0 if (phase in ("intro", "drive1", "block")) else 1
        route = s.initial if version == 0 else s.final
        index = step
        trav = route[: index + 1]
        self.traversed_line.set_data(*sc.route_xy(trav) if len(trav) > 1 else ([], []))
        self.traversed_line.set_alpha(1.0)
        if phase == "replan":
            k = max(1, int(round(draw_fraction * (len(s.new_remainder) - 1))))
            planned = s.new_remainder[: k + 1]
        else:
            planned = route[index:]
        self.planned_line.set_data(*sc.route_xy(planned) if len(planned) > 1 else ([], []))
        self.planned_line.set_alpha(1.0)
        self.previous_line.set_data(*sc.route_xy(s.abandoned) if version == 1 else ([], []))
        self.previous_line.set_alpha(1.0)
        self.reference_line.set_data(*sc.route_xy(s.reference["nodes"]))
        self.reference_line.set_alpha(reference_alpha)
        if not self.static:
            self.reference_label.set_alpha(reference_alpha)
            self.reference_label.get_bbox_patch().set_alpha(reference_alpha)
            self.reference_leader.set_alpha(reference_alpha)
        self._set_block(block_alpha)
        x, y = sc.xy[route[index]]
        self.vehicle.set_offsets([[x, y]])
        self.vehicle_ring.set_offsets([[x, y]])
        self.vehicle_ring.set_alpha(pulse)
        self.vehicle_ring.set_sizes([420 + 500 * (1 - pulse)])

        sm = s.summary
        new_cost = sm["new_remainder"]["cost"]
        old_cost = sm["abandoned_remainder"]["cost"]
        ref = s.reference
        if phase == "intro":
            msg = (f"Plan the cheapest route from the dock to the station: cost {sm['initial']['cost']:.1f} along aisle B. "
                   "Edges in the picking zone cost 3× their length.")
        elif phase == "drive1":
            msg = f"Following the planned route along aisle B, one graph node per step."
        elif phase == "block":
            msg = (f"Step {s.block_step}: {len(s.blocked)} nodes of aisle B are reported blocked, "
                   f"{sm['nodes_ahead_to_block']} nodes ahead of the navigator.")
        elif phase == "replan":
            msg = (f"Replanned from the current node: via aisle A, remaining cost {new_cost:.1f} (was {old_cost:.1f}); "
                   f"it avoids the picking zone.")
        elif phase == "drive2":
            msg = "Following the new route; the abandoned plan stays as a dashed reference."
        elif self.static:
            msg = (f"Blocked at step {s.block_step}, replanned from the navigator's node, arrived at step {s.steps}: "
                   f"cost {sm['traversed']['cost']:.1f} against {sm['initial']['cost']:.1f} planned.")
        else:
            msg = (f"Arrived after {s.steps} steps: travelled cost {sm['traversed']['cost']:.1f}, against "
                   f"{sm['initial']['cost']:.1f} planned before the blockage.")
        self.status.set_text(msg)
        self.counter.set_text(f"step {step:2d} / {s.steps}")
        self.tl_fill.set_data([0, step], [0, 0])
        self.tl_head.set_data([step], [0])
        travelled = s.cost_so_far(step)
        # The exported timeline records step 9 before the replan, so its remaining cost is the old one.
        remaining = new_cost if phase == "replan" else s.remaining(step)
        self.cost_text.set_text(f"travelled {travelled:5.1f}   remaining {remaining:5.1f}")

    def _set_block(self, alpha):
        t = self.t
        self.block_patch.set_alpha(0.22 * alpha)
        self.block_patch.set_edgecolor((*_rgb(t.blocked), alpha))
        self.block_marks.set_alpha(alpha)
        self.block_label.set_alpha(alpha)

    def _set_reset(self, r):
        """Arrival graphics fade out while the opening plan fades in; texts show the opening frame."""
        s, sc = self.s, self.s.scene
        self.set_state(step=0, phase="intro")  # texts, timeline and vehicle of the opening frame
        self.traversed_line.set_data(*sc.route_xy(s.final))
        self.traversed_line.set_alpha(1.0 - r)
        self.previous_line.set_data(*sc.route_xy(s.abandoned))
        self.previous_line.set_alpha(1.0 - r)
        self.planned_line.set_alpha(r)
        self._set_block(1.0 - r)

    def add_static_callouts(self):
        """Numbered notes for the static overview (not used in the animation)."""
        s, sc, t, ax = self.s, self.s.scene, self.t, self.ax
        sm = s.summary
        self.block_label.set_alpha(0.0)  # callout 2 says it
        notes = [
            (sc.xy[s.initial[5]], (2.4, 14.4), "1", f"plan: aisle B, cost {sm['initial']['cost']:.1f}"),
            (sc.xy[s.blocked[len(s.blocked) // 2]], (21.0, 13.75), "2",
             f"step {s.block_step}: {len(s.blocked)} nodes blocked"),
            (sc.xy[s.position], (2.4, 8.7), "3", f"replanned here, step {s.block_step}"),
            (sc.xy[s.reference["nodes"][len(s.reference["nodes"]) // 2]], (21.5, 21.9), "4",
             f"shortest by distance: {s.reference['length']:.1f} u, cost {s.reference['cost']:.1f}"),
        ]
        for (px, py), (tx, ty), num, text in notes:
            ax.annotate("", xy=(px, py), xytext=(tx, ty),
                        arrowprops=dict(arrowstyle="-", color=t.muted, lw=0.9, shrinkA=7, shrinkB=4), zorder=11)
            ax.text(tx, ty, f" {num} ", fontsize=9, color=t.background, ha="center", va="center", fontweight="bold",
                    zorder=13, bbox=dict(boxstyle="circle,pad=0.18", facecolor=t.text, edgecolor="none"))
            ax.text(tx + 0.9, ty, text, fontsize=9.5, color=t.text, ha="left", va="center", zorder=13,
                    bbox=dict(boxstyle="round,pad=0.2", facecolor=t.background, edgecolor="none", alpha=0.9))

    def image(self) -> Image.Image:
        self.fig.canvas.draw()
        buf = np.asarray(self.fig.canvas.buffer_rgba())
        return Image.fromarray(buf[..., :3].copy())


def _rgb(hex_color: str):
    h = hex_color.lstrip("#")
    return tuple(int(h[i:i + 2], 16) / 255 for i in (0, 2, 4))


def ease(x: float) -> float:
    return 0.5 - 0.5 * math.cos(math.pi * max(0.0, min(1.0, x)))


def frame_plan(story: Story):
    """(state kwargs, duration ms) for every frame; 50 ms per frame unless stated."""
    frames = []
    add = lambda n, **kw: frames.extend([(dict(kw), 50)] * n)  # noqa: E731
    add(34, step=0, phase="intro")
    for step in range(1, story.block_step + 1):
        add(5, step=step, phase="drive1")
    for k in range(10):
        frames.append((dict(step=story.block_step, phase="block", block_alpha=ease((k + 1) / 10),
                            pulse=1 - k / 10), 50))
    add(16, step=story.block_step, phase="block", block_alpha=1.0)
    for k in range(18):
        f = ease((k + 1) / 18)
        frames.append((dict(step=story.block_step, phase="replan", block_alpha=1.0, draw_fraction=f,
                            reference_alpha=0.9 * f), 50))
    add(34, step=story.block_step, phase="replan", block_alpha=1.0, draw_fraction=1.0, reference_alpha=0.9)
    for k in range(6):
        frames.append((dict(step=story.block_step, phase="drive2", block_alpha=1.0,
                            reference_alpha=0.9 * (1 - (k + 1) / 6)), 50))
    for step in range(story.block_step + 1, story.steps + 1):
        add(2, step=step, phase="drive2", block_alpha=1.0)
    add(40, step=story.steps, phase="arrive", block_alpha=1.0)
    for k in range(10):
        frames.append((dict(step=0, phase="intro", reset=ease((k + 1) / 10)), 50))
    return frames


def shared_palette(images, exact_colors):
    """One 256-colour palette for every frame (no flicker). Built from frames that hold all the
    colours, then the theme's key colours are written in exactly, so the background matches the
    page it sits on."""
    keys = [images[0], images[len(images) // 3], images[len(images) // 2], images[-15]]
    w, h = keys[0].size
    sheet = Image.new("RGB", (w, h * len(keys)))
    for k, im in enumerate(keys):
        sheet.paste(im, (0, k * h))
    adaptive = sheet.quantize(colors=256 - len(exact_colors), method=Image.Quantize.MEDIANCUT,
                              dither=Image.Dither.NONE)
    entries = adaptive.getpalette()[: 3 * (256 - len(exact_colors))]
    for color in exact_colors:
        entries += list(int(color.lstrip("#")[i:i + 2], 16) for i in (0, 2, 4))
    palette = Image.new("P", (1, 1))
    palette.putpalette(entries)
    return palette


def quantize_exact(image: Image.Image, palette: Image.Image) -> Image.Image:
    """Maps every pixel to its nearest palette colour exactly (Pillow's own conversion goes through
    a coarse colour cache, which turned pure white into a near-white)."""
    entries = np.array(palette.getpalette()[:768], dtype=np.int32).reshape(-1, 3)
    rgb = np.asarray(image, dtype=np.int32)
    keys = (rgb[..., 0] << 16) | (rgb[..., 1] << 8) | rgb[..., 2]
    unique, inverse = np.unique(keys.ravel(), return_inverse=True)
    colors = np.stack([(unique >> 16) & 255, (unique >> 8) & 255, unique & 255], axis=1)
    distance = ((colors[:, None, :] - entries[None, :, :]) ** 2).sum(axis=2)
    nearest = distance.argmin(axis=1).astype(np.uint8)
    indices = nearest[inverse].reshape(keys.shape)
    out = Image.fromarray(indices, mode="P")
    out.putpalette(palette.getpalette()[:768])
    return out


def render_static(story: Story, theme: str):
    frame = HeroFrame(story, theme, dpi=200, static=True)
    frame.set_state(step=story.steps, phase="arrive", block_alpha=1.0, reference_alpha=0.9)
    frame.add_static_callouts()
    path = ASSETS / f"hero-static-{theme}.png"
    frame.fig.savefig(path, dpi=200, facecolor=frame.fig.get_facecolor())
    plt.close(frame.fig)
    return path


def render_gif(story: Story, theme: str):
    frame = HeroFrame(story, theme, dpi=100)
    plan = frame_plan(story)
    images, durations = [], []
    for state, duration in plan:
        frame.set_state(**state)
        images.append(frame.image())
        durations.append(duration)
    plt.close(frame.fig)
    theme_colors = THEMES[theme]
    palette = shared_palette(images, [theme_colors.background, theme_colors.floor, theme_colors.text,
                                      theme_colors.muted, theme_colors.rack, theme_colors.route,
                                      theme_colors.route_light, theme_colors.blocked, theme_colors.marker,
                                      theme_colors.rule])
    quantized = [quantize_exact(im, palette) for im in images]
    path = ASSETS / f"hero-{theme}.gif"
    quantized[0].save(path, save_all=True, append_images=quantized[1:], duration=durations, loop=0,
                      optimize=False, disposal=1)
    return path, sum(durations) / 1000.0, len(images)


def main(themes=("light", "dark"), gif=True):
    ASSETS.mkdir(parents=True, exist_ok=True)
    story = Story()
    outputs = []
    for theme in themes:
        outputs.append(render_static(story, theme))
        if gif:
            path, seconds, count = render_gif(story, theme)
            outputs.append(path)
            print(f"  {path.name}: {count} frames, {seconds:.1f} s, {path.stat().st_size / 1e6:.2f} MB")
    return outputs


if __name__ == "__main__":
    main()

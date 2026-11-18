"""Procedural textures for the district scenes, generated at start (no image files, nothing downloaded).

Each texture is a small tile drawn from a fixed seed: brick in stretcher bond, stucco, Portland stone ashlar,
terracotta, slate, paving slabs, asphalt, glazing, grass, an iron railing with transparency, and the hatch
that marks where the modelled area ends. Tiles repeat in world units (see SCALE): a brick tile is one metre
of wall. Mipmaps and anisotropic filtering keep distant walls from shimmering.
"""
import math
import random

from panda3d.core import SamplerState, Texture

SIZE = 256
SCALE = {  # metres covered by one tile
    "asphalt": 4.0, "paving": 2.4, "stucco": 3.0, "stucco_rustic": 3.0, "brick_red": 1.0, "brick_stock": 1.0, "brick_brown": 1.0,
    "stone": 2.4, "terracotta": 1.6, "slate": 2.0, "grass": 6.0, "concrete": 3.0, "glass": 1.0, "railing": 1.0, "hatch": 3.0, "sett": 1.2,
}
_CACHE = {}


def _clamp(v):
    return 0 if v < 0 else (255 if v > 255 else int(v))


def _image(name, fn, alpha=False):
    rng = random.Random(name)
    w = h = SIZE
    ch = 4 if alpha else 3
    buf = bytearray(w * h * ch)
    noise = [rng.random() for _ in range(64 * 64)]

    def smooth(x, y, cells=16):
        # Value noise on a periodic 64x64 lattice: tiles seamlessly.
        fx, fy = x / w * cells, y / h * cells
        x0, y0 = int(fx) % cells, int(fy) % cells
        x1, y1 = (x0 + 1) % cells, (y0 + 1) % cells
        tx, ty = fx - int(fx), fy - int(fy)
        tx, ty = tx * tx * (3 - 2 * tx), ty * ty * (3 - 2 * ty)
        n = lambda i, j: noise[(j % 64) * 64 + (i % 64)]  # noqa: E731
        a = n(x0, y0) + (n(x1, y0) - n(x0, y0)) * tx
        b = n(x0, y1) + (n(x1, y1) - n(x0, y1)) * tx
        return a + (b - a) * ty

    for y in range(h):
        for x in range(w):
            px = fn(x, y, rng, smooth)
            i = (y * w + x) * ch
            buf[i] = _clamp(px[2] * 255)  # Panda3D keeps RAM images in BGR(A) order
            buf[i + 1] = _clamp(px[1] * 255)
            buf[i + 2] = _clamp(px[0] * 255)
            if alpha:
                buf[i + 3] = _clamp(px[3] * 255)
    tex = Texture(name)
    tex.setup2dTexture(w, h, Texture.TUnsignedByte, Texture.FRgba if alpha else Texture.FRgb)
    tex.setRamImage(bytes(buf))
    tex.setWrapU(SamplerState.WMRepeat)
    tex.setWrapV(SamplerState.WMRepeat)
    tex.setMinfilter(SamplerState.FTLinearMipmapLinear)
    tex.setMagfilter(SamplerState.FTLinear)
    tex.setAnisotropicDegree(8)
    return tex


def _tint(base, v, k):
    return tuple(c * (1.0 - k + 2.0 * k * v) for c in base)


def _bricks(base, mortar, courses=13, per_row=4):
    """Stretcher bond: `courses` per tile height, `per_row` bricks per tile width, joints of mortar."""
    def fn(x, y, rng, smooth):
        ch = SIZE / courses
        row = int(y // ch)
        off = 0.5 * (row % 2)
        bw = SIZE / per_row
        col = int((x / bw + off) // 1)
        jy = (y % ch) < 1.6
        jx = ((x / bw + off) % 1.0) * bw < 1.6
        if jx or jy:
            return _tint(mortar, smooth(x, y, 32), 0.06)
        r = random.Random(row * 131 + col * 7 + int(base[0] * 100)).random()
        return _tint(base, 0.35 * r + 0.65 * smooth(x, y, 32), 0.16)
    return fn


def _flat(base, k=0.05, cells=16):
    return lambda x, y, rng, smooth: _tint(base, smooth(x, y, cells), k)


def _stucco_rustic(x, y, rng, smooth):
    base = (0.92, 0.90, 0.84)
    band = (y % (SIZE / 7.5)) < 2.0  # rusticated ground floor: horizontal channels every 0.4 m
    c = _tint(base, smooth(x, y, 16), 0.03)
    return tuple(v * 0.78 for v in c) if band else c


def _ashlar(x, y, rng, smooth):
    base = (0.84, 0.82, 0.76)  # Portland stone
    ch = SIZE / 4.0
    row = int(y // ch)
    off = 0.37 * (row % 2)
    joint = (y % ch) < 1.2 or ((x / (SIZE / 2.0) + off) % 1.0) * (SIZE / 2.0) < 1.2
    c = _tint(base, smooth(x, y, 24), 0.07)
    return tuple(v * 0.86 for v in c) if joint else c


def _terracotta(x, y, rng, smooth):
    base = (0.88, 0.66, 0.53)  # pinky-buff Doulton terracotta (Historic England list entry 1294346)
    blocks = (y % (SIZE / 5.0)) < 1.5 or (x % (SIZE / 3.0)) < 1.5
    rel = 0.5 + 0.5 * math.sin(x * 0.25) * math.sin(y * 0.25)  # fine moulded relief
    c = _tint(base, 0.6 * smooth(x, y, 32) + 0.4 * rel, 0.10)
    return tuple(v * 0.88 for v in c) if blocks else c


def _slate(x, y, rng, smooth):
    base = (0.29, 0.31, 0.34)
    course = (y % (SIZE / 8.0)) < 1.5
    c = _tint(base, smooth(x, y, 32), 0.10)
    return tuple(v * 0.75 for v in c) if course else c


def _paving(x, y, rng, smooth):
    base = (0.66, 0.66, 0.64)
    sw, sh = SIZE / 4.0, SIZE / (2.4 / 0.9)  # 600 x 900 mm slabs in a 2.4 m tile
    row = int(y // sh)
    off = 0.5 * (row % 2)
    joint = (y % sh) < 1.3 or ((x / sw + off) % 1.0) * sw < 1.3
    r = random.Random(row * 31 + int(x / sw + off)).random()
    c = _tint(base, 0.4 * r + 0.6 * smooth(x, y, 24), 0.07)
    return tuple(v * 0.80 for v in c) if joint else c


def _sett(x, y, rng, smooth):
    base = (0.42, 0.41, 0.40)  # granite setts in a mews
    sw, sh = SIZE / 10.0, SIZE / 12.0
    row = int(y // sh)
    off = 0.5 * (row % 2)
    joint = (y % sh) < 1.5 or ((x / sw + off) % 1.0) * sw < 1.5
    r = random.Random(row * 17 + int(x / sw + off)).random()
    c = _tint(base, r, 0.18)
    return tuple(v * 0.6 for v in c) if joint else c


def _asphalt(x, y, rng, smooth):
    v = 0.55 * smooth(x, y, 32) + 0.25 * smooth(x, y, 8) + 0.20 * rng.random()
    return _tint((0.215, 0.22, 0.235), v, 0.12)


def _glass(x, y, rng, smooth):
    # A sash window's glazing: sky reflected near the top, darker below, glazing bars dividing the panes.
    t = 1.0 - y / SIZE
    sky = (0.55 + 0.25 * t, 0.62 + 0.22 * t, 0.70 + 0.18 * t)
    dark = (0.10, 0.12, 0.15)
    k = min(1.0, max(0.0, 0.25 + 0.75 * t ** 1.6 + 0.08 * (smooth(x, y, 8) - 0.5)))
    c = tuple(d + (s - d) * k * 0.55 for d, s in zip(dark, sky))
    bar = (abs(x - SIZE / 2) < 2.0) or (abs(y - SIZE / 2) < 2.5)
    return (0.90, 0.90, 0.86) if bar else c


def _railing(x, y, rng, smooth):
    # Black cast-iron railing: bars every 12 cm with spear heads, top and bottom rails; transparent between.
    u = x / SIZE
    v = 1.0 - y / SIZE
    bar = (u * 8.0) % 1.0 < 0.16
    head = bar and v > 0.90 and ((u * 8.0) % 1.0 < 0.16 - (v - 0.90) * 1.2)
    rail = abs(v - 0.86) < 0.018 or v < 0.04
    on = (bar and v <= 0.90) or head or rail
    return (0.05, 0.06, 0.06, 1.0 if on else 0.0)


def _hatch(x, y, rng, smooth):
    on = ((x + y) % 32) < 7
    return (0.78, 0.70, 0.30) if on else (0.36, 0.37, 0.36)


GENERATORS = {
    "asphalt": (_asphalt, False), "paving": (_paving, False), "sett": (_sett, False),
    "stucco": (_flat((0.93, 0.91, 0.85), 0.03), False), "stucco_rustic": (_stucco_rustic, False),
    "brick_red": (_bricks((0.58, 0.25, 0.19), (0.72, 0.68, 0.62)), False),
    "brick_stock": (_bricks((0.70, 0.58, 0.40), (0.70, 0.67, 0.60)), False),
    "brick_brown": (_bricks((0.44, 0.33, 0.26), (0.62, 0.59, 0.54)), False),
    "stone": (_ashlar, False), "terracotta": (_terracotta, False), "slate": (_slate, False),
    "concrete": (_flat((0.70, 0.69, 0.66), 0.08, 24), False), "grass": (_flat((0.36, 0.50, 0.27), 0.16, 32), False),
    "glass": (_glass, False), "railing": (_railing, True), "hatch": (_hatch, False),
}


def texture(name):
    tex = _CACHE.get(name)
    if tex is None:
        fn, alpha = GENERATORS[name]
        tex = _CACHE[name] = _image(name, fn, alpha)
    return tex


def scale(name):
    return SCALE.get(name, 1.0)

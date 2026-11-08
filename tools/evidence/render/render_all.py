"""Render every README figure, light and dark, from docs/data."""

from __future__ import annotations

import time

import diagram
import failure
import hero
import perf
import search


def main(themes=("light", "dark"), gif=True):
    started = time.perf_counter()
    outputs = []
    outputs += hero.main(themes=themes, gif=gif)
    outputs += search.main(themes=themes)
    outputs += perf.main(themes=themes)
    outputs += failure.main(themes=themes)
    outputs += diagram.main(themes=themes)
    for path in outputs:
        print(f"  {path.name}  {path.stat().st_size / 1e3:,.0f} kB")
    print(f"rendered {len(outputs)} files in {time.perf_counter() - started:.1f} s")
    return outputs


if __name__ == "__main__":
    main()

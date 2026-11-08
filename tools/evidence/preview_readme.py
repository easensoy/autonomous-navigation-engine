#!/usr/bin/env python3
"""Render README.md to a local HTML page styled like GitHub's README view, for checking layout,
<picture> light/dark switching and mermaid diagrams before pushing.

    python tools/evidence/preview_readme.py        # writes build/readme-preview/index.html
    python -m http.server 8765 --bind 127.0.0.1    # from the repository root, then open
    # http://127.0.0.1:8765/build/readme-preview/index.html

Relative image paths resolve against the repository root, so serve the root (or pass --file to use a
file:// base instead). Switch the OS or browser colour scheme to see the dark variants.
Needs the `markdown` package (pip install markdown). The page loads mermaid from jsDelivr.
"""

from __future__ import annotations

import html
import re
from pathlib import Path

import markdown

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "build" / "readme-preview" / "index.html"

STYLE = """
:root { color-scheme: light dark; --bg:#ffffff; --fg:#1f2328; --muted:#59636e; --border:#d1d9e0;
        --code:#f6f8fa; --link:#0969da; }
@media (prefers-color-scheme: dark) {
  :root { --bg:#0d1117; --fg:#f0f6fc; --muted:#9198a1; --border:#3d444d; --code:#151b23; --link:#4493f8; }
}
html, body { background: var(--bg); color: var(--fg); margin: 0; }
body { font: 16px/1.5 -apple-system, BlinkMacSystemFont, "Segoe UI", "Noto Sans", Helvetica, Arial, sans-serif; }
.frame { max-width: 1012px; margin: 24px auto; padding: 0 16px; }
.box { border: 1px solid var(--border); border-radius: 6px; }
.box-header { padding: 8px 16px; border-bottom: 1px solid var(--border); font-size: 14px; font-weight: 600; }
article { padding: 32px; max-width: 980px; overflow-wrap: break-word; }
@media (max-width: 767px) { article { padding: 16px; } .frame { padding: 0 8px; margin: 8px auto; } }
h1, h2 { border-bottom: 1px solid var(--border); padding-bottom: .3em; margin-top: 24px; margin-bottom: 16px; }
h1 { font-size: 2em; font-weight: 600; } h2 { font-size: 1.5em; font-weight: 600; }
h3 { font-size: 1.25em; font-weight: 600; margin-top: 24px; }
p, ul, table, pre { margin-top: 0; margin-bottom: 16px; }
a { color: var(--link); text-decoration: none; } a:hover { text-decoration: underline; }
img { max-width: 100%; box-sizing: content-box; }
code { font: 85% ui-monospace, SFMono-Regular, Consolas, monospace; background: var(--code); padding: .2em .4em;
       border-radius: 6px; }
pre { background: var(--code); padding: 16px; overflow: auto; border-radius: 6px; font-size: 85%; line-height: 1.45; }
pre code { background: none; padding: 0; font-size: 100%; }
table { border-collapse: collapse; display: block; width: max-content; max-width: 100%; overflow: auto; }
th, td { border: 1px solid var(--border); padding: 6px 13px; }
tr:nth-child(2n) { background: var(--code); }
sub { font-size: 75%; color: var(--fg); }
.mermaid { background: none; text-align: center; }
"""


def render(file_base: bool = False) -> Path:
    text = (REPO / "README.md").read_text(encoding="utf-8")
    # mermaid blocks become <pre class="mermaid"> (GitHub renders them natively).
    blocks = {}

    def stash(match):
        key = f"MERMAIDBLOCK{len(blocks)}"
        blocks[key] = f'<pre class="mermaid">{html.escape(match.group(1))}</pre>'
        return key

    text = re.sub(r"```mermaid\n(.*?)```", stash, text, flags=re.S)
    body = markdown.markdown(text, extensions=["tables", "fenced_code", "md_in_html"])
    for key, block in blocks.items():
        body = body.replace(f"<p>{key}</p>", block).replace(key, block)
    base = REPO.as_uri() + "/" if file_base else "/"
    page = f"""<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>README preview</title><base href="{base}"><style>{STYLE}</style></head>
<body><div class="frame"><div class="box"><div class="box-header">README.md</div><article>
{body}
</article></div></div>
<script type="module">
import mermaid from "https://cdn.jsdelivr.net/npm/mermaid@11.4.1/dist/mermaid.esm.min.mjs";
const dark = window.matchMedia("(prefers-color-scheme: dark)").matches;
mermaid.initialize({{ startOnLoad: true, theme: dark ? "dark" : "default" }});
</script></body></html>"""
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(page, encoding="utf-8")
    return OUT


if __name__ == "__main__":
    import sys

    print(render(file_base="--file" in sys.argv))

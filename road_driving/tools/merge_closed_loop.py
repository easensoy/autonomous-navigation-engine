"""Merge closed-loop campaign parts run one process per scenario into the files one process writes.

    python merge_closed_loop.py OUT_DIR PART_DIR [PART_DIR ...]

Each PART_DIR holds the closed_loop.jsonl and closed_loop.md of `road_drive bench-closed --scenario NAME`.
Give the parts in the split's scenario order: closed_loop.jsonl is their concatenation in that order, as one
process writes it. closed_loop.md has one section per scenario, which one process writes in name order, so
the sections are sorted by name; the text before the first section and after the last must be identical
in every part. Episodes are independent (each scenario's runs, the reference included, happen in its own
part), so the merged files equal those of a single process.
"""
import os
import sys

FOOTER = "Clearance is the smallest distance"


def split_md(text):
    lines = text.splitlines(keepends=True)
    first = next(i for i, l in enumerate(lines) if l.startswith("## "))
    last = next(i for i, l in enumerate(lines) if l.startswith(FOOTER))
    sections, name = {}, None
    for l in lines[first:last]:
        if l.startswith("## "):
            name = l[3:].strip()
            sections[name] = []
        sections[name].append(l)
    return "".join(lines[:first]), {k: "".join(v) for k, v in sections.items()}, "".join(lines[last:])


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    out, parts = sys.argv[1], sys.argv[2:]
    jsonl, head, foot, sections = [], None, None, {}
    for p in parts:
        with open(os.path.join(p, "closed_loop.jsonl"), "rb") as f:
            jsonl.append(f.read())
        with open(os.path.join(p, "closed_loop.md"), "r", encoding="utf-8", newline="") as f:
            h, s, t = split_md(f.read())
        if head is None:
            head, foot = h, t
        elif h != head or t != foot:
            sys.exit("%s: header or footer differs from the first part's" % p)
        for k, v in s.items():
            if k in sections:
                sys.exit("scenario %s appears in more than one part" % k)
            sections[k] = v
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "closed_loop.jsonl"), "wb") as f:
        f.write(b"".join(jsonl))
    with open(os.path.join(out, "closed_loop.md"), "w", encoding="utf-8", newline="") as f:
        f.write(head + "".join(sections[k] for k in sorted(sections)) + foot)
    print("merged %d parts, %d scenarios -> %s" % (len(parts), len(sections), out))


if __name__ == "__main__":
    main()

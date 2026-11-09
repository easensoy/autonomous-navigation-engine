#!/usr/bin/env python3
"""Packs campaigns' rosbag2 recordings into one archive per campaign and scenario, for upload as GitHub
release assets, and lists every archive and every bag file with its size and SHA-256 in
ros2/evidence/recordings.json.

  python ros2/tools/hash_recordings.py --campaigns <name> [<name> ...] [--release-tag <tag>]

The recordings stay out of Git (build/ros2-trials/ is ignored); recordings.json is committed, so anyone
who downloads the assets can check them. Standard library only.
"""

import argparse
import hashlib
import json
import tarfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
BUILD = REPO / "build" / "ros2-trials"


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--campaigns", nargs="+", required=True)
    parser.add_argument("--release-tag", help="tag of the GitHub release the archives were uploaded to")
    args = parser.parse_args()

    out_dir = BUILD / "release"
    out_dir.mkdir(parents=True, exist_ok=True)
    assets = []
    for campaign in args.campaigns:
        for scenario_dir in sorted(p for p in (BUILD / campaign).iterdir() if p.is_dir()):
            bags = sorted(scenario_dir.glob("trial_*/bag"))
            if not bags:
                continue
            archive = out_dir / f"{campaign}_{scenario_dir.name}_bags.tar"
            with tarfile.open(archive, "w") as tar:
                for bag in bags:
                    tar.add(bag, arcname=f"{campaign}/{scenario_dir.name}/{bag.parent.name}/bag")
            members = [{"path": f"{campaign}/{scenario_dir.name}/{m.parent.parent.name}/bag/{m.name}",
                        "bytes": m.stat().st_size, "sha256": sha256(m)}
                       for m in sorted(scenario_dir.glob("trial_*/bag/*.mcap"))]
            assets.append({"asset": archive.name, "bytes": archive.stat().st_size, "sha256": sha256(archive),
                           "contains": members})
    manifest = {
        "schema": "ane-sim/recordings/1",
        "campaigns": args.campaigns,
        "release_tag": args.release_tag,
        "location": (f"https://github.com/easensoy/autonomous-navigation-engine/releases/tag/{args.release_tag}"
                     if args.release_tag else None),
        "published": bool(args.release_tag),
        "status": (f"uploaded as assets of the GitHub release {args.release_tag}" if args.release_tag else
                   "not published: the archives exist only on the machine that ran the trials "
                   "(build/ros2-trials/release); this file records their sizes and SHA-256 so that a later "
                   "upload can be checked against them"),
        "format": "rosbag2, MCAP storage with zstd chunk compression; one tar archive per campaign and scenario",
        "note": "Not in Git. Each archive's SHA-256 is listed, and the SHA-256 of every bag file inside it.",
        "assets": assets,
    }
    (REPO / "ros2" / "evidence" / "recordings.json").write_text(json.dumps(manifest, indent=1) + "\n", encoding="utf-8")
    total = sum(a["bytes"] for a in assets)
    print(f"{len(assets)} archives, {total / 1e6:.1f} MB, in {out_dir}")


if __name__ == "__main__":
    main()

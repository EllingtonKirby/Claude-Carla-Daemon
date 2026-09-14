#!/usr/bin/env python3
"""Split a multi-route leaderboard XML into one file per route.

    python tools/split_routes.py IN.xml OUT_DIR            # writes OUT_DIR/route_<id>.xml
    python tools/split_routes.py IN.xml --list             # print id, town, scenario types

One evaluator process drives one route file, so probe sessions take single-route XMLs. Stdlib only.
"""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("routes_xml", type=Path)
    ap.add_argument("out_dir", type=Path, nargs="?")
    ap.add_argument("--list", action="store_true", help="list routes instead of writing files")
    args = ap.parse_args()

    root = ET.parse(args.routes_xml).getroot()
    routes = root.findall("route")
    if args.list:
        for r in routes:
            scen = r.find("scenarios")
            types = [s.get("type") for s in scen] if scen is not None else []
            print(f"{r.get('id')}\t{r.get('town')}\t{','.join(types)}")
        return 0
    if args.out_dir is None:
        ap.error("out_dir is required unless --list")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    for r in routes:
        out = ET.Element("routes")
        out.append(r)
        path = args.out_dir / f"route_{r.get('id')}.xml"
        ET.ElementTree(out).write(path, encoding="utf-8", xml_declaration=True)
    print(f"wrote {len(routes)} route file(s) to {args.out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

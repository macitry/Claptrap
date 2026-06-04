#!/usr/bin/env python3
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


def add_imu_sites(xml_path: Path) -> int:
    tree = ET.parse(xml_path)
    root = tree.getroot()
    added = 0

    for body in root.iter("body"):
        name = body.get("name", "")
        if not name.startswith("imu_"):
            continue

        site_name = f"{name}_site"
        if any(child.tag == "site" and child.get("name") == site_name for child in body):
            continue

        body.append(
            ET.Element(
                "site",
                {
                    "name": site_name,
                    "pos": "0 0 0",
                    "size": "0.005",
                },
            )
        )
        added += 1

    ET.indent(tree, space="  ")
    xml_path.write_text(ET.tostring(root, encoding="unicode") + "\n")
    return added


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Add colocated MuJoCo sites to IMU marker bodies."
    )
    parser.add_argument("xml", type=Path)
    args = parser.parse_args()

    if not args.xml.exists():
        parser.error(f"XML file not found: {args.xml}")

    added = add_imu_sites(args.xml)
    print(f"Added {added} IMU site(s): {args.xml}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

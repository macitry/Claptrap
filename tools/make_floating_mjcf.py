#!/usr/bin/env python3
import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def parse_vec3(value):
    parts = value.split()
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected three numbers, for example: '0 0 0.35'")
    for part in parts:
        float(part)
    return value


def parse_vec4(value):
    parts = value.split()
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("expected four numbers, for example: '0.999048 0.043619 0 0'")
    for part in parts:
        float(part)
    return value


def main():
    parser = argparse.ArgumentParser(
        description="Wrap a generated MJCF worldbody in a free-floating root body."
    )
    parser.add_argument("input_xml", type=Path)
    parser.add_argument("output_xml", type=Path)
    parser.add_argument("--body-name", default="floating_base")
    parser.add_argument("--joint-name", default="root_free")
    parser.add_argument("--root-pos", default="0 0 0.35", type=parse_vec3)
    parser.add_argument(
        "--root-quat",
        default="0.999048 0.043619 0 0",
        type=parse_vec4,
        help="initial floating base quaternion; default is a 5 degree roll perturbation",
    )
    args = parser.parse_args()

    if not args.input_xml.exists():
        print(f"Input XML not found: {args.input_xml}", file=sys.stderr)
        return 1

    tree = ET.parse(args.input_xml)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        print("Input XML has no <worldbody> section", file=sys.stderr)
        return 1

    children = list(worldbody)
    if not children:
        print("Input XML <worldbody> is empty", file=sys.stderr)
        return 1

    wrapper = ET.Element(
        "body",
        {"name": args.body_name, "pos": args.root_pos, "quat": args.root_quat},
    )
    wrapper.append(ET.Element("freejoint", {"name": args.joint_name}))

    for child in children:
        worldbody.remove(child)
        wrapper.append(child)

    worldbody.append(wrapper)
    ET.indent(tree, space="  ")

    args.output_xml.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output_xml, encoding="unicode")
    args.output_xml.write_text(args.output_xml.read_text() + "\n")
    print(f"Saved floating MJCF: {args.output_xml}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

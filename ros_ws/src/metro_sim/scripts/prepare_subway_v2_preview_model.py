#!/usr/bin/env python3
# Copyright 2026 jo0625
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import argparse
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


def local_name(tag: str) -> str:
    """Return an XML tag without its optional namespace."""
    return tag.rsplit("}", 1)[-1]


def main() -> None:
    """Create a sensor-free robot model for visual and collision inspection."""
    parser = argparse.ArgumentParser()
    parser.add_argument("source_sdf", type=Path)
    parser.add_argument("source_config", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()

    tree = ET.parse(args.source_sdf)
    root = tree.getroot()
    model = next(element for element in root if local_name(element.tag) == "model")
    model.set("name", "subway_v2_collision_preview")

    for parent in root.iter():
        for child in list(parent):
            if local_name(child.tag) in {"sensor", "plugin"}:
                parent.remove(child)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(args.output_dir / "model.sdf", encoding="utf-8", xml_declaration=True)
    shutil.copy2(args.source_config, args.output_dir / "model.config")


if __name__ == "__main__":
    main()

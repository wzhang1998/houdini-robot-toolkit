"""Register this repo's Houdini package, so Houdini loads the toolkit's
assets at every start.

    python scripts/install_houdini_package.py              # newest Documents/houdiniXX.X
    python scripts/install_houdini_package.py --dir "C:/Users/me/Documents/houdini22.0"
    python scripts/install_houdini_package.py --dry-run

The package is houdini/houdini_robot_toolkit.json, in the repo, with paths
relative to itself ($HOUDINI_PACKAGE_PATH): it puts otls/ on
HOUDINI_OTLSCAN_PATH (robot_arm, the CSV I/O asset robot_arm needs inside
it, dance_phrase). This writes one pointer to it,
<houdini user dir>/packages/houdini_robot_toolkit_path.json =
{"package_path": "<repo>/houdini"}. Re-run it if the repo moves; restart
Houdini after. Alternative with no file in the Houdini user dir: set the
environment variable HOUDINI_PACKAGE_DIR=<repo>/houdini before Houdini
starts (every Houdini version).

Without it an asset installed for one session only is missing after a
restart, and a scene falls back to the copy embedded in the .hip -- the CSV
I/O asset's embedded copy has no Python module, so Retime / Export / Import
fail with KeyError 'PythonModule'.
"""

import argparse
import glob
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))).replace("\\", "/")
NAME = "houdini_robot_toolkit_path.json"
OLD = "houdini_robot_toolkit.json"      # the earlier full copy (absolute paths), replaced by the pointer


def pointer(root=ROOT):
    return {"package_path": root + "/houdini"}


def user_dir():
    """The newest Documents/houdiniXX.X (not *_OLD and the like)."""
    docs = os.path.join(os.path.expanduser("~"), "Documents")
    found = [d for d in glob.glob(os.path.join(docs, "houdini*")) if re.fullmatch(r"houdini\d+\.\d+", os.path.basename(d))]
    if not found:
        raise SystemExit("no Documents/houdiniXX.X found; pass --dir")
    return max(found, key=lambda d: tuple(int(x) for x in re.findall(r"\d+", os.path.basename(d))))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", help="Houdini user directory (default: the newest Documents/houdiniXX.X)")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)
    pkgs = os.path.join(a.dir or user_dir(), "packages")
    path = os.path.join(pkgs, NAME)
    text = json.dumps(pointer(), indent=4)
    print(path)
    print(text)
    if a.dry_run:
        return 0
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text + "\n")
    old = os.path.join(pkgs, OLD)
    if os.path.exists(old) and "HOUDINI_ROBOT_TOOLKIT" in open(old).read():
        os.remove(old)                           # else both would load the same otls/
        print("removed the earlier copy", old)
    print("written -- restart Houdini")
    return 0


if __name__ == "__main__":
    sys.exit(main())

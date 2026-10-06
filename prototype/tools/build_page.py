#!/usr/bin/env python3
"""Inject data into the web templates.

Two pages are built from this directory:

  lab      web/lab_template.html + web/parity_fixture.json -> web/drishti_lab.html
           The interactive one. You draw the terrain and the system drives it.
           The fixture is 520 C++ decisions the page re-runs on load to prove
           its JavaScript still agrees with the shipping cores.

  console  web/template.html + runs.json -> web/drishti_console.html
           The older replay of four fixed scenarios. Needs export_runs.py to
           have written runs.json first.

    python tools/build_page.py                 # both, skipping any missing input
    python tools/build_page.py --target lab
    python tools/export_runs.py && python tools/build_page.py --target console

An earlier version built only the console and ignored --out, so asking it for
the lab page silently produced the console under the lab's name. Targets are
named now, and an unbuildable target is an error rather than a wrong file.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROTO = os.path.dirname(HERE)
WEB = os.path.join(PROTO, "web")

TARGETS = {
    "lab": {
        "template": os.path.join(WEB, "lab_template.html"),
        "data": os.path.join(WEB, "parity_fixture.json"),
        "placeholder": "__FIXTURE__",
        "out": os.path.join(WEB, "drishti_lab.html"),
        "make": "tools/parity_fixture.cpp",
    },
    "console": {
        "template": os.path.join(WEB, "template.html"),
        "data": os.path.join(PROTO, "runs.json"),
        "placeholder": "__RUNS_JSON__",
        "out": os.path.join(WEB, "drishti_console.html"),
        "make": "tools/export_runs.py",
    },
}


def build(name, spec, out_override=None):
    for path in (spec["template"], spec["data"]):
        if not os.path.exists(path):
            raise SystemExit("%s: missing %s -- run %s first"
                             % (name, os.path.relpath(path, PROTO), spec["make"]))

    template = open(spec["template"], encoding="utf-8").read()
    data = open(spec["data"], encoding="utf-8").read().strip()

    # The one failure that would break the page silently: a closing script tag
    # inside the JSON ends the data block early, and everything after it is
    # parsed as markup.
    if "</script" in data.lower():
        raise SystemExit("%s: data contains a closing script tag" % name)
    if spec["placeholder"] not in template:
        raise SystemExit("%s: template has no %s placeholder"
                         % (name, spec["placeholder"]))

    json.loads(data)          # refuse to embed something the page cannot parse

    out = out_override or spec["out"]
    page = template.replace(spec["placeholder"], data)
    open(out, "w", encoding="utf-8").write(page)
    print("%-8s -> %s  (%.1f KB)"
          % (name, os.path.relpath(out, PROTO), len(page) / 1024.0))
    return json.loads(data)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--target", choices=["lab", "console", "all"], default="all")
    ap.add_argument("--out", help="override the output path (single target only)")
    args = ap.parse_args(argv)

    if args.out and args.target == "all":
        raise SystemExit("--out needs a single --target")

    names = list(TARGETS) if args.target == "all" else [args.target]
    built = 0
    for name in names:
        spec = TARGETS[name]
        # A bare "all" is a convenience, so a console with no recorded runs is
        # skipped rather than fatal. Asking for it by name still fails loudly.
        if args.target == "all" and not os.path.exists(spec["data"]):
            print("%-8s .. skipped, no %s"
                  % (name, os.path.relpath(spec["data"], PROTO)))
            continue
        data = build(name, spec, args.out)
        built += 1
        if name == "lab":
            print("         %d supervisor + %d terrain cases embedded"
                  % (len(data["supervisor"]), len(data["terrain"])))
        else:
            for r in data["runs"]:
                print("         %-8s %-14s %3d frames  %s"
                      % (r["id"], r["world"]["name"], len(r["frames"]), r["outcome"]))

    if not built:
        raise SystemExit("nothing built")
    return 0


if __name__ == "__main__":
    sys.exit(main())

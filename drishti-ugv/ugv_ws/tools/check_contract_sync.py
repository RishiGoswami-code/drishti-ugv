#!/usr/bin/env python3
"""Cross-file consistency checks that no single compiler can catch.

Five things drift silently in this workspace and each one is a real hazard:

  1. drishti_msgs/msg/SafetyState.msg constants vs the C++ Action/Reason enums.
     A mismatch means the supervisor publishes a reason code that means
     something else downstream -- the audit trail lies.

  2. The Params defaults in supervisor_core.hpp vs drishti_bringup's
     drishti.yaml. A mismatch means a node started without a params file
     silently runs a different safety envelope than one started with it.

  3. Every SPEC.md section 9.3 parameter is present in both.

  4. Traversability weights and limits: traversability_core.hpp vs
     traversability.yaml. Same failure as 2, for terrain cost.

  5. Cross-package agreement on two rules that are stated in more than one
     place:
       - SPEC.md 6.2 ("unknown terrain is expensive, never free") is encoded
         both as traversability unknown_cost and as the taxonomy UNKNOWN tier
         cost. An unobserved cell and an unclassified pixel must cost the same.
       - t_camera_stale and t_depth_stale are read by both the supervisor and
         the perception node. If they disagree, one stops while the other
         still reports healthy.

Runs anywhere with Python 3; needs neither ROS nor a compiler.

    python tools/check_contract_sync.py
"""
import os
import re
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MSG = os.path.join(WS, "src", "drishti_msgs", "msg", "SafetyState.msg")
HPP = os.path.join(WS, "src", "drishti_safety", "include", "drishti_safety",
                   "supervisor_core.hpp")
YAML = os.path.join(WS, "src", "drishti_bringup", "config", "drishti.yaml")
TRAV_HPP = os.path.join(WS, "src", "drishti_traversability", "include",
                        "drishti_traversability", "traversability_core.hpp")
TRAV_YAML = os.path.join(WS, "src", "drishti_bringup", "config",
                         "traversability.yaml")
TAXONOMY = os.path.join(WS, "src", "drishti_perception", "drishti_perception",
                        "taxonomy.py")
PERCEPTION_YAML = os.path.join(WS, "src", "drishti_bringup", "config",
                               "perception.yaml")

# SPEC.md section 9.3
SPEC_PARAMS = {
    "t_camera_stale", "t_depth_stale", "d_emergency", "c_critical",
    "v_max", "v_slow", "cov_max", "watchdog_period",
    "t_frame_static",
}

failures = []


def fail(msg):
    failures.append(msg)
    print("  FAIL  " + msg)


def ok(msg):
    print("  ok    " + msg)


def read(path):
    with open(path, encoding="utf-8") as fh:
        return fh.read()


def msg_constants(text, prefix):
    """ACTION_/REASON_ constants from a .msg file, stripped of their prefix."""
    out = {}
    for m in re.finditer(r"^\s*uint8\s+(%s\w+)\s*=\s*(\d+)" % prefix, text, re.M):
        out[m.group(1)[len(prefix):]] = int(m.group(2))
    return out


def cpp_enum(text, name):
    """Values of an `enum class <name> : std::uint8_t { ... }` block."""
    m = re.search(r"enum\s+class\s+%s\s*:[^{]*\{(.*?)\}" % name, text, re.S)
    if not m:
        fail("could not find enum class %s in supervisor_core.hpp" % name)
        return {}
    # Strip comments BEFORE splitting: a doc comment may itself contain a comma
    # ("clamped to v_slow, ..."), which would otherwise split mid-comment and
    # glue the tail onto the next enumerator.
    body = re.sub(r"/\*.*?\*/", "", m.group(1), flags=re.S)
    body = re.sub(r"//[^\n]*", "", body)

    out = {}
    for chunk in body.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        em = re.match(r"^(\w+)\s*=\s*(\d+)$", chunk)
        if em:
            out[em.group(1)] = int(em.group(2))
        else:
            fail("could not parse enumerator %r in enum class %s" % (chunk, name))
    return out


def cpp_param_defaults(text):
    """Params struct member defaults: `double name{1.23};`"""
    m = re.search(r"struct\s+Params\s*\{(.*?)\n\};", text, re.S)
    if not m:
        fail("could not find struct Params in supervisor_core.hpp")
        return {}
    return {n: float(v) for n, v in
            re.findall(r"double\s+(\w+)\s*\{\s*([-\d.eE+]+)\s*\}", m.group(1))}


def yaml_supervisor_params(text):
    """Numeric params under safety_supervisor: ros__parameters:."""
    m = re.search(r"^safety_supervisor:\s*\n\s+ros__parameters:\s*\n(.*)", text,
                  re.S | re.M)
    if not m:
        fail("could not find safety_supervisor block in drishti.yaml")
        return {}
    out = {}
    for line in m.group(1).splitlines():
        if not line.strip() or line.strip().startswith("#"):
            continue
        if not line.startswith("    "):        # left the block
            break
        pm = re.match(r"\s+(\w+)\s*:\s*([-\d.eE+]+)\s*(?:#.*)?$", line)
        if pm:
            out[pm.group(1)] = float(pm.group(2))
    return out


msg_text, hpp_text, yaml_text = read(MSG), read(HPP), read(YAML)

print("1. SafetyState.msg constants vs C++ enums")
for prefix, enum_name in (("ACTION_", "Action"), ("REASON_", "Reason")):
    from_msg = msg_constants(msg_text, prefix)
    from_cpp = cpp_enum(hpp_text, enum_name)
    if not from_msg:
        fail("no %s constants found in SafetyState.msg" % prefix)
        continue
    if from_msg == from_cpp:
        ok("%-6s %d constants match exactly" % (enum_name, len(from_msg)))
    else:
        for k in sorted(set(from_msg) | set(from_cpp)):
            a, b = from_msg.get(k), from_cpp.get(k)
            if a != b:
                fail("%s.%s: .msg=%s C++=%s" % (enum_name, k, a, b))

print("2. Params defaults: supervisor_core.hpp vs drishti.yaml")
cpp = cpp_param_defaults(hpp_text)
yml = yaml_supervisor_params(yaml_text)
for name in sorted(SPEC_PARAMS):
    c, y = cpp.get(name), yml.get(name)
    if c is None:
        fail("%s missing from struct Params" % name)
    elif y is None:
        fail("%s missing from drishti.yaml" % name)
    elif abs(c - y) > 1e-12:
        fail("%s: header default=%g, yaml=%g" % (name, c, y))
    else:
        ok("%-16s %g" % (name, c))

print("3. SPEC.md section 9.3 coverage")
missing_cpp = SPEC_PARAMS - set(cpp)
missing_yml = SPEC_PARAMS - set(yml)
if missing_cpp:
    fail("SPEC 9.3 params absent from header: %s" % sorted(missing_cpp))
if missing_yml:
    fail("SPEC 9.3 params absent from yaml: %s" % sorted(missing_yml))
if not missing_cpp and not missing_yml:
    ok("all %d SPEC 9.3 parameters present in both" % len(SPEC_PARAMS))

print("4. Traversability weights and limits: header vs traversability.yaml")


def cpp_struct_defaults(text, struct):
    """double/bool members with brace-initialised defaults in a struct."""
    m = re.search(r"struct\s+%s\s*\{(.*?)\n\};" % struct, text, re.S)
    if not m:
        fail("could not find struct %s in traversability_core.hpp" % struct)
        return {}
    return {n: float(v) for n, v in
            re.findall(r"double\s+(\w+)\s*\{\s*([-\d.eE+]+)\s*\}", m.group(1))}


trav_hpp = read(TRAV_HPP)
trav_yaml = read(TRAV_YAML)
try:
    import yaml as _yaml
    trav_params = _yaml.safe_load(trav_yaml)["traversability_fusion"]["ros__parameters"]
except Exception as exc:                                    # noqa: BLE001
    fail("could not parse traversability.yaml: %s" % exc)
    trav_params = {}

for struct, section in (("Weights", "weights"), ("Limits", "limits")):
    cpp_vals = cpp_struct_defaults(trav_hpp, struct)
    yaml_vals = trav_params.get(section, {})
    if not cpp_vals:
        continue
    if not yaml_vals:
        fail("traversability.yaml has no '%s' section" % section)
        continue
    for name, cval in sorted(cpp_vals.items()):
        yval = yaml_vals.get(name)
        if yval is None:
            fail("%s.%s missing from traversability.yaml" % (section, name))
        elif abs(float(yval) - cval) > 1e-12:
            fail("%s.%s: header default=%g, yaml=%g" % (section, name, cval, yval))
        else:
            ok("%-8s %-20s %g" % (section, name, cval))
    extra = set(yaml_vals) - set(cpp_vals)
    if extra:
        fail("traversability.yaml %s has keys the header does not: %s"
             % (section, sorted(extra)))

print("5. SPEC 6.2 across packages, and staleness across nodes")

# The cost of ignorance is stated in three places, and all three are the same
# rule. If they drift, an unobserved terrain cell, an unclassified pixel and an
# unrecognised class id get priced differently for no defensible reason.
trav_unknown = trav_params.get("limits", {}).get("unknown_cost")
tax_unknown = None
m = re.search(r"Tier\.UNKNOWN:\s*TierPolicy\(cost=([\d.]+)", read(TAXONOMY))
if m:
    tax_unknown = float(m.group(1))
else:
    fail("could not find the UNKNOWN tier cost in taxonomy.py")

if trav_unknown is not None and tax_unknown is not None:
    if abs(float(trav_unknown) - tax_unknown) > 1e-12:
        fail("SPEC 6.2 drift: traversability unknown_cost=%g but the taxonomy "
             "UNKNOWN tier costs %g. Both encode 'unknown terrain is expensive, "
             "never free'." % (trav_unknown, tax_unknown))
    else:
        ok("unknown cost agrees across packages: %g" % tax_unknown)

# The supervisor and perception must agree about what "fresh" means, or one
# stops while the other still reports healthy.
try:
    import yaml as _yaml
    perc = _yaml.safe_load(read(PERCEPTION_YAML))["perception"]["ros__parameters"]
except Exception as exc:                                    # noqa: BLE001
    fail("could not parse perception.yaml: %s" % exc)
    perc = {}

for key in ("t_camera_stale", "t_depth_stale"):
    sup_val, perc_val = yml.get(key), perc.get(key)
    if sup_val is None or perc_val is None:
        fail("%s missing from drishti.yaml or perception.yaml" % key)
    elif abs(float(sup_val) - float(perc_val)) > 1e-12:
        fail("%s: supervisor=%g, perception=%g -- the two disagree about "
             "freshness" % (key, sup_val, perc_val))
    else:
        ok("%-16s agrees between supervisor and perception: %g" % (key, sup_val))

print()
if failures:
    print("%d PROBLEM(S)" % len(failures))
    sys.exit(1)
print("contract in sync")

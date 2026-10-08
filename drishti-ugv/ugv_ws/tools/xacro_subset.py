"""Resolve the small xacro subset this workspace uses, without ROS or xacro.

Handles <xacro:property>, ${...} expressions and <xacro:macro> with parameters,
which is all drishti.urdf.xacro needs. Shared by the offline checkers so each of
them sees the same expansion.
"""
import re
from xml.etree import ElementTree as ET

XACRO_NS = "http://www.ros.org/wiki/xacro"


def evaluate(expr, props):
    """Evaluate a ${...} body against the property table.

    Identifiers are replaced by repr() of their value, so a string property
    becomes a quoted literal rather than a bare name eval() would reject --
    that is what lets ${name}_link expand inside a macro.
    """
    def swap(m):
        key = m.group(1)
        return repr(props[key]) if key in props else key

    resolved = re.sub(r"\b([A-Za-z_]\w*)\b", swap, expr)
    try:
        # Arithmetic and string literals only; the description uses nothing else.
        return eval(resolved, {"__builtins__": {}}, {})  # noqa: S307
    except Exception:
        return None


def substitute(text, props):
    if text is None:
        return None
    out, guard = text, 0
    while "${" in out and guard < 10:
        guard += 1
        new = ""
        i = 0
        while i < len(out):
            if out.startswith("${", i):
                depth, j = 1, i + 2
                while j < len(out) and depth:
                    if out[j] == "{":
                        depth += 1
                    elif out[j] == "}":
                        depth -= 1
                    j += 1
                val = evaluate(out[i + 2:j - 1], props)
                new += out[i:j] if val is None else (
                    repr(round(val, 12)) if isinstance(val, float) else str(val))
                i = j
            else:
                new += out[i]
                i += 1
        if new == out:
            break
        out = new
    return out


def expand(node, props, out, on_error=print):
    """Walk the xacro tree, expanding properties and macros into `out`."""
    for child in node:
        tag = child.tag
        if tag == "{%s}property" % XACRO_NS:
            raw = child.get("value")
            val = substitute(raw, props)
            try:
                props[child.get("name")] = float(val)
            except (TypeError, ValueError):
                props[child.get("name")] = val
            continue
        if tag == "{%s}macro" % XACRO_NS:
            props.setdefault("__macros__", {})[child.get("name")] = child
            continue
        if tag.startswith("{%s}" % XACRO_NS):
            name = tag.split("}")[1]
            macro = props.get("__macros__", {}).get(name)
            if macro is None:
                on_error("unknown xacro element: %s" % name)
                continue
            local = dict(props)
            for param in (macro.get("params") or "").split():
                local[param] = substitute(child.get(param), props)
                try:
                    local[param] = float(local[param])
                except (TypeError, ValueError):
                    pass
            expand(macro, local, out, on_error)
            continue

        clone = ET.Element(tag, {k: substitute(v, props) for k, v in child.attrib.items()})
        # Element text carries real content here -- <gz_frame_id>camera_left_optical
        # </gz_frame_id> is a text node, and dropping it made check 6 vacuous.
        clone.text = substitute(child.text, props)
        clone.tail = child.tail
        out.append(clone)
        expand(child, props, clone, on_error)

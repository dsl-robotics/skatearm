"""Natural-language -> rbt program for Skate Commander (offline-first).

Turn a plain request ("raise both arms, then home") into lines of the existing
``rbt`` API (see program.py). This NEVER moves the robot — it only returns code
text for the editor, which still runs through the sandboxed ProgramRunner
(Click-to-Step, collision guard, E-STOP, joint limits). So the risk surface is
small and safety stays downstream.

Engines:
  local   deterministic intent parser (default; offline, no keys, can only ever
          emit known rbt.* calls)
  llm     optional fallback used only when SKATE_NL_LLM is on and an API key is
          set; its output is passed through the SAME ast validator before it is
          ever accepted.

Public API:
  generate(text)  -> {"code", "engine"} on success, else {"error", "hint"}
  validate(code)  -> (ok: bool, reason: str)

No robot imports here (only ast/re/os) so it is trivially unit-testable.
"""
from __future__ import annotations

import ast
import os
import re

# rbt methods the validator will accept (mirrors program.RobotAPI).
ALLOWED = {"movej", "pose", "movel", "moveto", "home", "gripper", "waypoint",
           "wait", "tcp", "q", "status"}

# Gripper open/close angles (deg); the bridge clamps to real limits anyway.
GRIP_OPEN, GRIP_CLOSE = 40, 0
# "raise arms" canonical pose (deg): shoulder abduction (J2) + elbow (J4).
RAISE_ABDUCT, RAISE_ELBOW = 30, 80


# ===========================================================================
# AST safety validator — makes ANY generated code safe to load
# ===========================================================================
def validate(code):
    """Return (ok, reason). Allows only rbt.<known>(...) calls with literal
    args (and {joint: deg} dicts) plus bounded `for x in range(int...)` loops."""
    try:
        tree = ast.parse(code, "<nl>", "exec")
    except SyntaxError as e:
        return False, f"syntax error: {e.msg}"
    for node in tree.body:
        ok, why = _stmt_ok(node)
        if not ok:
            return False, why
    return True, ""


def _stmt_ok(node):
    if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call):
        return _call_ok(node.value)
    if isinstance(node, ast.For):
        if not (isinstance(node.target, ast.Name)
                and isinstance(node.iter, ast.Call)
                and isinstance(node.iter.func, ast.Name)
                and node.iter.func.id == "range"
                and 1 <= len(node.iter.args) <= 3
                and all(isinstance(a, ast.Constant) and isinstance(a.value, int)
                        for a in node.iter.args)
                and not node.iter.keywords and not node.orelse):
            return False, "only `for _ in range(int, ...)` loops are allowed"
        for b in node.body:
            ok, why = _stmt_ok(b)
            if not ok:
                return False, why
        return True, ""
    if isinstance(node, ast.Pass):
        return True, ""
    return False, ("only rbt.* calls and bounded for-loops are allowed "
                   f"(got {type(node).__name__})")


def _call_ok(call):
    f = call.func
    if not (isinstance(f, ast.Attribute) and isinstance(f.value, ast.Name)
            and f.value.id == "rbt"):
        return False, "only rbt.<method>(...) calls are allowed"
    if f.attr not in ALLOWED:
        return False, f"unknown rbt method: {f.attr!r}"
    for a in call.args:
        ok, why = _arg_ok(a)
        if not ok:
            return False, why
    for kw in call.keywords:
        if kw.arg is None:
            return False, "**kwargs are not allowed"
        ok, why = _arg_ok(kw.value)
        if not ok:
            return False, why
    return True, ""


def _arg_ok(a):
    if isinstance(a, ast.Constant) and isinstance(a.value, (int, float, str, bool)):
        return True, ""
    if (isinstance(a, ast.UnaryOp) and isinstance(a.op, ast.USub)
            and isinstance(a.operand, ast.Constant)
            and isinstance(a.operand.value, (int, float))):
        return True, ""
    if isinstance(a, ast.Dict):
        for k, v in zip(a.keys, a.values):
            if not (isinstance(k, ast.Constant) and isinstance(k.value, str)):
                return False, "pose dict keys must be joint strings"
            ok, why = _arg_ok(v)
            if not ok:
                return False, why
        return True, ""
    return False, "arguments must be plain numbers, strings, or a {joint: deg} dict"


# ===========================================================================
# Local intent parser
# ===========================================================================
_WORDS = {"a": 1, "an": 1, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
          "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10, "once": 1,
          "twice": 2, "eleven": 11, "twelve": 12, "fifteen": 15, "twenty": 20,
          "thirty": 30, "forty": 40, "fifty": 50, "sixty": 60, "ninety": 90}


def _num(tok):
    if tok is None:
        return None
    t = str(tok).strip().lower()
    if t in _WORDS:
        return float(_WORDS[t])
    try:
        return float(t)
    except ValueError:
        return None


def _g(x):
    """Format a number without a trailing .0 (5.0 -> '5')."""
    return f"{x:g}"


def _arm(cl, default=None):
    has_l, has_r = "left" in cl, "right" in cl
    if has_l and has_r:
        return "both"
    if has_l:
        return "left"
    if has_r:
        return "right"
    return default


def _arms_in(cl):
    if "both" in cl:
        return ["left", "right"]
    out = []
    if "left" in cl:
        out.append("left")
    if "right" in cl:
        out.append("right")
    return out


def _repeat(seg):
    """Pull a trailing repeat count off a segment: returns (n, segment)."""
    m = re.search(r"\b(\d+|[a-z]+)\s*(?:times|x)\s*$", seg, re.I)
    if m:
        n = _num(m.group(1))
        if n and n >= 2:
            return int(n), seg[:m.start()].strip()
    m = re.search(r"\b(twice)\s*$", seg, re.I)
    if m:
        return 2, seg[:m.start()].strip()
    return 1, seg


def _resolve_joint(cl):
    """Phrase -> list of joint tokens ('L4', 'R2', 'H1'). None if no joint."""
    toks = [m.group(1).upper() + m.group(2)
            for m in re.finditer(r"\b([lr])\s*([1-8])\b", cl)]
    for m in re.finditer(r"\bh\s*([12])\b", cl):
        toks.append("H" + m.group(1))
    if toks:
        return toks
    m = re.search(r"\bj(?:oint)?\s*([1-8])\b", cl)
    if m:
        return [f"L{m.group(1)}", f"R{m.group(1)}"]
    if "head" in cl:
        return ["H1"]
    base = None
    for word, jn in (("elbow", 4), ("abduction", 2), ("shoulder", 2),
                     ("wrist", 6)):
        if word in cl:
            base = jn
            break
    if base is None:
        return None
    arm = _arm(cl, "both")
    sides = ["L", "R"] if arm == "both" else [{"left": "L", "right": "R"}[arm]]
    return [f"{s}{base}" for s in sides]


def _pose_line(pairs):
    body = ", ".join(f"'{t}': {_g(v)}" for t, v in pairs)
    return f"rbt.pose({{{body}}})"


# direction -> (axis, sign); world axes match the cockpit's cartesian jog.
_DIRS = {"up": ("dz", 1), "down": ("dz", -1), "forward": ("dy", 1),
         "forwards": ("dy", 1), "back": ("dy", -1), "backward": ("dy", -1),
         "backwards": ("dy", -1), "in": ("dy", 1), "out": ("dy", -1),
         "leftward": ("dx", -1), "rightward": ("dx", 1)}


def _clause(c):
    """One clause -> list of rbt lines, or None if not understood."""
    cl = " " + c.lower().strip() + " "

    # home / rest
    if re.search(r"\b(home|go home|return home|rest position|stand by|reset)\b", cl):
        return ["rbt.home()"]

    # wait / pause
    if re.search(r"\b(wait|pause|dwell|sleep|hold)\b", cl):
        m = re.search(r"\b(?:wait|pause|dwell|sleep|hold)\b\s*(?:for\s*)?"
                      r"([\w.]+)?\s*(seconds?|secs?|s|minutes?|mins?|m)?", cl)
        n = _num(m.group(1)) if (m and m.group(1)) else 1.0
        if n is None:
            n = 1.0
        if m and m.group(2) and m.group(2).startswith("min"):
            n *= 60
        return [f"rbt.wait({_g(n)})"]

    # waypoint
    m = re.search(r"way\s*point\s+([\w-]+)", cl)
    if m:
        w = m.group(1)
        arg = w if re.fullmatch(r"\d+", w) else repr(w)
        return [f"rbt.waypoint({arg})"]

    # gripper
    m = re.search(r"\b(open|close|grip|release|grab|let go)\b", cl)
    if m and re.search(r"\b(grippers?|hands?|claw)\b", cl):
        opening = m.group(1) in ("open", "release", "let go")
        deg = GRIP_OPEN if opening else GRIP_CLOSE
        arms = _arms_in(cl) or ["right"]
        return [f"rbt.gripper('{a}', {deg})" for a in arms]

    # wave (composite, uses a small loop)
    if "wave" in cl:
        side = {"left": "L", "right": "R"}.get(_arm(cl, "right"), "R")
        return [_pose_line([(f"{side}2", 30), (f"{side}4", 60)]),
                "for _ in range(2):",
                f"    rbt.movej('{side}4', 95)",
                f"    rbt.movej('{side}4', 55)"]

    # raise / lower arms -> coordinated pose
    m = re.search(r"\b(raise|lift|lower|drop|put down)\b", cl)
    if m and re.search(r"\barms?\b", cl):
        up = m.group(1) in ("raise", "lift")
        arm = _arm(cl, "both")
        sides = ["L", "R"] if arm == "both" else [{"left": "L", "right": "R"}[arm]]
        ab, el = (RAISE_ABDUCT, RAISE_ELBOW) if up else (0, 15)
        pairs = []
        for s in sides:
            pairs += [(f"{s}2", ab), (f"{s}4", el)]
        return [_pose_line(pairs)]

    # cartesian: move a hand up/down/forward/back ... by N mm/cm
    if re.search(r"\b(move|nudge|shift|jog|step|raise|lower)\b", cl) \
            and re.search(r"\b(hand|arm|tcp|gripper|it)\b", cl):
        arm = _arm(cl, "right")
        if arm == "both":
            arm = "right"
        direction = None
        for word, val in _DIRS.items():
            if re.search(rf"\b{word}\b", cl):
                direction = val
                break
        if direction is None:  # "raise/lower the hand" with no explicit dir
            if re.search(r"\b(raise|lift)\b", cl):
                direction = ("dz", 1)
            elif re.search(r"\b(lower|drop)\b", cl):
                direction = ("dz", -1)
        if direction is not None:
            mnum = re.search(r"(-?\d+(?:\.\d+)?)\s*(mm|cm|m|millimeters?|"
                             r"centimeters?|meters?)?", cl)
            dist = float(mnum.group(1)) if mnum else 50.0
            unit = (mnum.group(2) or "mm") if mnum else "mm"
            if unit.startswith(("cm", "centi")):
                dist *= 10
            elif unit in ("m", "meter", "meters"):
                dist *= 1000
            axis, sign = direction
            return [f"rbt.movel('{arm}', {axis}={_g(sign * dist)})"]

    # move a joint to an absolute angle
    if re.search(r"\bto\b", cl) or re.search(r"\b(bend|set|rotate|turn)\b", cl):
        joints = _resolve_joint(cl)
        m = re.search(r"\bto\b\s*(-?[\w.]+)\s*(deg|degrees|°)?", cl) \
            or re.search(r"(-?\d+(?:\.\d+)?)\s*(deg|degrees|°)", cl)
        if joints and m:
            ang = _num(m.group(1))
            if ang is not None:
                if len(joints) == 1:
                    return [f"rbt.movej('{joints[0]}', {_g(ang)})"]
                return [_pose_line([(t, ang) for t in joints])]

    return None


def parse_local(text):
    """Whole request -> (list_of_lines, '') or (None, offending_clause)."""
    raw = (text or "").strip()
    if not raw:
        return None, "(empty)"
    out = []
    segments = re.split(r"\s*(?:;|\n|\bthen\b)\s*", raw, flags=re.I)
    for seg in segments:
        seg = seg.strip().strip(".").strip()
        if not seg:
            continue
        n, seg = _repeat(seg)
        seg_lines = []
        for clause in re.split(r"\s*(?:,|\band\b)\s*", seg, flags=re.I):
            clause = clause.strip()
            if not clause:
                continue
            lines = _clause(clause)
            if lines is None:
                return None, clause
            seg_lines += lines
        if not seg_lines:
            return None, seg
        if n >= 2:
            out.append(f"for _ in range({n}):")
            out += ["    " + ln for ln in seg_lines]
        else:
            out += seg_lines
    return (out, "") if out else (None, raw)


# ===========================================================================
# Optional LLM fallback (off unless SKATE_NL_LLM + a key are set)
# ===========================================================================
_LLM_SYS = (
    "You translate a plain-language request into a short program for a bimanual "
    "robot, using ONLY this Python API (degrees; cartesian in mm; world axes):\n"
    "  rbt.movej('L4', deg)   one joint: L1..L8 / R1..R8 (L4/R4 elbow, "
    "L2/R2 shoulder, L8/R8 gripper), H1..H2 head\n"
    "  rbt.pose({'L4': deg, 'R4': deg})   several joints at once\n"
    "  rbt.movel('left'|'right', dx=, dy=, dz=)   step the TCP in mm\n"
    "  rbt.home()   rbt.gripper('left'|'right', deg)   rbt.waypoint(i|'name')   "
    "rbt.wait(seconds)\n"
    "Bounded `for _ in range(n):` loops are allowed. Output ONLY the code, no "
    "prose, no markdown fences, no imports, no other names.")


def _strip_fences(s):
    s = s.strip()
    if s.startswith("```"):
        s = re.sub(r"^```[a-zA-Z]*\n?", "", s)
        s = re.sub(r"\n?```$", "", s)
    return s.strip()


def parse_llm(text):
    if os.environ.get("SKATE_NL_LLM", "").lower() in ("", "0", "off", "no", "false"):
        return None
    key = os.environ.get("ANTHROPIC_API_KEY")
    model = os.environ.get("SKATE_NL_MODEL", "claude-3-5-haiku-latest")
    if not key:
        return None
    try:
        import httpx
        r = httpx.post(
            "https://api.anthropic.com/v1/messages",
            headers={"x-api-key": key, "anthropic-version": "2023-06-01",
                     "content-type": "application/json"},
            json={"model": model, "max_tokens": 700, "system": _LLM_SYS,
                  "messages": [{"role": "user", "content": text}]},
            timeout=20.0)
        r.raise_for_status()
        out = "".join(p.get("text", "") for p in r.json().get("content", [])
                      if p.get("type") == "text")
        code = _strip_fences(out)
        ok, _why = validate(code)
        return code if (ok and code.strip()) else None
    except Exception:
        return None


# ===========================================================================
# Public entry point
# ===========================================================================
def generate(text):
    lines, bad = parse_local(text)
    if lines:
        code = "# from: " + " ".join((text or "").split()) + "\n" + "\n".join(lines) + "\n"
        ok, why = validate(code)
        if ok:
            return {"code": code, "engine": "local"}
        # local should always be valid; treat as a bug, fall through to hint
    code = parse_llm(text)
    if code:
        return {"code": "# from: " + " ".join((text or "").split())
                + "  (LLM)\n" + code.rstrip() + "\n", "engine": "llm"}
    return {"error": f"couldn't turn that into a program (stuck on: {bad!r})",
            "hint": "Try e.g. 'raise both arms, then home', "
                    "'bend the left elbow to 40 degrees', "
                    "'move the right hand up 5 cm', "
                    "'open the left gripper', 'wave the right hand twice'."}


if __name__ == "__main__":
    import json
    tests = [
        "raise both arms, then home",
        "bend the left elbow to 40 degrees",
        "move the right hand up 5 cm",
        "open the left gripper",
        "wave the right hand twice",
        "set L4 to 70 then wait 2 seconds then home",
        "lower the arms",
        "go to waypoint 2",
        "move the left hand forward 30 mm and close the left gripper",
        "do a backflip",            # should fail -> hint
    ]
    for t in tests:
        print("»", t)
        print(json.dumps(generate(t), ensure_ascii=False, indent=0))
        print("-" * 60)

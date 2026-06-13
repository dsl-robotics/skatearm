"""Unit tests for nl.py — the NL -> rbt validator + local parser.

Run standalone (`python3 test_nl.py`) or under pytest. No robot deps.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "skate_commander"))
import nl  # noqa: E402


def test_validator_rejects_danger():
    bad = [
        "import os",
        "__import__('os').system('x')",
        "open('/etc/passwd').read()",
        "rbt.movej('L4', 40)\nos.system('x')",
        "while True:\n    rbt.home()",
        "def f():\n    pass",
        "rbt.evil()",
        "print('hi')",
        "x = 5",
        "rbt.movej(foo, 40)",
        "[rbt.home() for _ in range(3)]",
        "rbt.home().__class__",
        "for x in [1, 2]:\n    rbt.home()",   # iter not range()
        "lambda: rbt.home()",
    ]
    for c in bad:
        ok, why = nl.validate(c)
        assert not ok, f"should REJECT but accepted: {c!r}"


def test_validator_accepts_good():
    good = [
        "rbt.home()",
        "rbt.movej('L4', 40)",
        "rbt.movej('R4', -10)",
        "rbt.pose({'L4': 80, 'R4': 80})",
        "rbt.movel('right', dz=50)",
        "rbt.movel('left', dx=-30, dz=10)",
        "rbt.gripper('left', 0)",
        "rbt.wait(2)",
        "rbt.waypoint(2)",
        "rbt.waypoint('pick')",
        "for _ in range(2):\n    rbt.movej('R4', 95)\n    rbt.movej('R4', 55)",
    ]
    for c in good:
        ok, why = nl.validate(c)
        assert ok, f"should ACCEPT but rejected: {c!r} ({why})"


def test_generate_examples():
    cases = {
        "raise both arms": "rbt.pose({'L2': 30, 'L4': 80, 'R2': 30, 'R4': 80})",
        "bend the left elbow to 40 degrees": "rbt.movej('L4', 40)",
        "set R2 to 25": "rbt.movej('R2', 25)",
        "move the right hand up 5 cm": "rbt.movel('right', dz=50)",
        "open the left gripper": "rbt.gripper('left', 40)",
        "close both grippers": "rbt.gripper('left', 0)",
        "home": "rbt.home()",
        "wait 2 seconds": "rbt.wait(2)",
        "go to waypoint 3": "rbt.waypoint(3)",
    }
    for text, expect in cases.items():
        r = nl.generate(text)
        assert "code" in r, (text, r)
        assert expect in r["code"], (text, r["code"])
        assert nl.validate(r["code"])[0], (text, r["code"])


def test_sequences_and_repeat():
    r = nl.generate("set L4 to 70 then wait 1 second then home")
    code = r["code"]
    assert code.index("movej('L4', 70)") < code.index("wait(1)") < code.index("home()")
    r2 = nl.generate("wave the right hand twice")
    assert "for _ in range(2):" in r2["code"]
    assert nl.validate(r2["code"])[0]


def test_unknown_gives_hint():
    r = nl.generate("do a backflip")
    assert "error" in r and "hint" in r and "code" not in r


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print("ok ", fn.__name__)
    print(f"ALL {len(fns)} PASS")

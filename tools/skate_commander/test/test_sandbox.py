"""The rbt program runner AST-validates user code before exec: imports and
dunder name/attribute access (the exec-sandbox escape vectors) are rejected,
while ordinary robot programs pass. Hardware-free (no bridge / MuJoCo).

    python -m pytest -q tools/skate_commander/test/test_sandbox.py
"""
import ast
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "skate_ros2"))

from skate_commander.program import _Sandbox, _SandboxError   # noqa: E402


@pytest.mark.parametrize("src", [
    "().__class__.__base__.__subclasses__()",
    "[].__class__.__bases__[0].__subclasses__()",
    "import os",
    "from os import system",
    "x = __import__('os')",
    "b = __builtins__",
    "g = (lambda: 0).__globals__",
    "object.__subclasses__(object)",
])
def test_sandbox_rejects_escapes(src):
    with pytest.raises(_SandboxError):
        _Sandbox().visit(ast.parse(src, "<t>", "exec"))


def test_sandbox_allows_normal_program():
    ok = (
        "for i in range(3):\n"
        "    rbt.movej('L4', 10 + i)\n"
        "    if rbt.ok() and not rbt.blocked():\n"
        "        rbt.wait(0.3)\n"
        "total = sum([1, 2, 3])\n"
        "rbt.pose({'L2': 20, 'R2': -20})\n"
        "x = math.sin(0.5)\n"
    )
    _Sandbox().visit(ast.parse(ok, "<t>", "exec"))   # must not raise

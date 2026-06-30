# Security Policy

SkateArm is a sim-first research / portfolio project rather than production software.
Even so, because it speaks a real robot's control protocol, safety and security are
taken seriously.

## Reporting a vulnerability

Please **do not** open a public issue for security-sensitive problems. Instead, email
**porche121004@gmail.com** with:

- a description of the issue and its impact,
- steps to reproduce, and
- any suggested fix.

You can expect an acknowledgement within a few days. Once a fix is ready, you'll be
credited (if you wish) in the release notes.

## Areas of particular interest

- The UDP control protocol (`tools/skate_ros2/`) — packet spoofing, a missed deadman,
  or malformed-packet handling.
- The cockpit server (`tools/skate_commander/`) — the sandboxed `rbt` program executor
  and WebSocket handling.
- Anything that could let a remote command bypass the collision guard or the E-STOP.

## Threat model

The `skate_ros2` wire speaks the Skate firmware's **native UDP protocol, which serializes
with Python `pickle`**. `pickle` is unsafe against untrusted input — a crafted packet is an
arbitrary-code-execution primitive. Treat the control link accordingly:

- Run it only on a **trusted, isolated LAN** (or loopback) — never expose the control port
  to the public internet.
- There is currently **no authentication** on the wire; anyone who can reach the port can
  command motion (subject to the firmware deadman / E-STOP).
- The UDP **decoder is hardened**: `decode_packet` uses a restricted unpickler with an
  **exact** allow-list — the telemetry classes plus numpy array reconstruction only, *no*
  `numpy.*` wildcard — so a crafted packet can't reach `os.system` / `eval` / `numpy.f2py`.
  Set `SKATE_WIRE=raw` to opt out.
- The cockpit's `rbt` program runner **AST-validates** user code before running it (rejects
  imports and dunder name / attribute access) on top of restricted builtins, blocking the
  usual `exec`-sandbox escapes. It is still a local-tool guard, not a hostile-tenant boundary.
- The cockpit binds `127.0.0.1` by default, and its WebSocket **refuses cross-site origins**
  (a DNS-rebinding defense).

Transport **authentication** (so only an authorised client can command motion) is still
tracked on the roadmap; until then, keep the link on a trusted LAN.

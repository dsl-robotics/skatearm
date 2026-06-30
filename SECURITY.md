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

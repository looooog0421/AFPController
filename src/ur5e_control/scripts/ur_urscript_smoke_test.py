#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""URScript TCP smoke tests for URSim/UR5e.

Purpose:
- Safely validate that (1) TCP connectivity works and (2) robot moves as expected.
- joint_wiggle: move one joint at a time to catch joint-order mismatches.
- line_move: small cartesian move to validate TCP / direction.

This script talks to the URScript "secondary" interface (default port 30002)
by sending URScript text over TCP.

NOTE:
- Use URSim first.
- Keep speeds/accelerations low.
- Ensure the robot/URSim is in Remote Control (if required).

Example:
  python3 ur_urscript_smoke_test.py --robot-ip 192.168.10.103 --mode joint_wiggle --tcp-z 0.221
  python3 ur_urscript_smoke_test.py --robot-ip 192.168.10.103 --mode line_move --tcp-z 0.221
"""

import argparse
import socket
import time
from dataclasses import dataclass


DEFAULT_SAFE_Q = [
    0.0,
    -1.57,
    1.57,
    -1.57,
    -1.57,
    0.0,
]


@dataclass
class URParams:
    a: float = 0.5  # joint/cartesian acceleration (conservative)
    v: float = 0.2  # joint/cartesian speed (conservative)
    blend: float = 0.0


def _send_urscript(host: str, port: int, script: str, timeout_s: float = 2.0):
    # UR expects \n terminated lines; sending one program string is common.
    payload = script
    if not payload.endswith("\n"):
        payload += "\n"

    with socket.create_connection((host, port), timeout=timeout_s) as s:
        s.sendall(payload.encode("utf-8"))


def _fmt_list(values):
    return "[" + ",".join(f"{v:.6f}" for v in values) + "]"


def make_program_set_tcp(tcp_z: float):
    # Only set translation first; keep rotation zero to avoid frame confusion.
    return f"set_tcp(p[0,0,{tcp_z:.6f},0,0,0])\n"


def make_program_joint_wiggle(q0, delta_rad=0.0872664626, p: URParams = URParams()):
    # delta_rad default ~= 5deg
    lines = [
        "def smoke_joint_wiggle():",
        "  set_safety_mode_transition_hardness(1)",
        f"  movej({_fmt_list(q0)}, a={p.a:.3f}, v={p.v:.3f})",
        "  sleep(0.5)",
    ]

    for i in range(6):
        q = list(q0)
        q[i] += delta_rad
        lines += [
            f"  movej({_fmt_list(q)}, a={p.a:.3f}, v={p.v:.3f})",
            "  sleep(0.3)",
            f"  movej({_fmt_list(q0)}, a={p.a:.3f}, v={p.v:.3f})",
            "  sleep(0.3)",
        ]

    lines += [
        "end",
        "smoke_joint_wiggle()",
    ]
    return "\n".join(lines) + "\n"


def make_program_line_move(pose_start=None, dx=0.05, p: URParams = URParams()):
    # If pose_start is None, use get_actual_tcp_pose() from robot.
    # Move along +X in base frame by dx, then back.
    lines = [
        "def smoke_line_move():",
        "  set_safety_mode_transition_hardness(1)",
    ]

    if pose_start is None:
        lines += [
            "  p0 = get_actual_tcp_pose()",
        ]
    else:
        # pose is [x,y,z,rx,ry,rz]
        lines += [
            f"  p0 = p{_fmt_list(pose_start)}",
        ]

    lines += [
        f"  p1 = pose_trans(p0, p[{dx:.6f},0,0,0,0,0])",
        f"  movel(p0, a={p.a:.3f}, v={p.v:.3f}, r={p.blend:.3f})",
        "  sleep(0.2)",
        f"  movel(p1, a={p.a:.3f}, v={p.v:.3f}, r={p.blend:.3f})",
        "  sleep(0.2)",
        f"  movel(p0, a={p.a:.3f}, v={p.v:.3f}, r={p.blend:.3f})",
        "end",
        "smoke_line_move()",
    ]

    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-ip", default="192.168.10.103")
    ap.add_argument("--port", type=int, default=30002)
    ap.add_argument("--mode", choices=["joint_wiggle", "line_move"], required=True)
    ap.add_argument("--tcp-z", type=float, default=0.221)
    ap.add_argument("--a", type=float, default=0.5)
    ap.add_argument("--v", type=float, default=0.2)
    ap.add_argument("--sleep", type=float, default=0.0, help="Optional extra sleep after sending program")
    ap.add_argument("--delta-deg", type=float, default=5.0)
    ap.add_argument("--dx", type=float, default=0.05)
    ap.add_argument("--use-safe-q", action="store_true", help="Pre-move to a conservative joint config (DEFAULT_SAFE_Q)")
    args = ap.parse_args()

    params = URParams(a=args.a, v=args.v)

    # Program string
    program = ""
    program += make_program_set_tcp(args.tcp_z)

    if args.mode == "joint_wiggle":
        delta_rad = args.delta_deg * 3.141592653589793 / 180.0
        q0 = DEFAULT_SAFE_Q if args.use_safe_q else DEFAULT_SAFE_Q
        program += make_program_joint_wiggle(q0=q0, delta_rad=delta_rad, p=params)
    else:
        program += make_program_line_move(dx=args.dx, p=params)

    print(f"Sending URScript to {args.robot_ip}:{args.port} | mode={args.mode} tcp_z={args.tcp_z}")
    _send_urscript(args.robot_ip, args.port, program)

    if args.sleep > 0:
        time.sleep(args.sleep)

    print("Done. Observe motion in URSim / robot.")


if __name__ == "__main__":
    main()

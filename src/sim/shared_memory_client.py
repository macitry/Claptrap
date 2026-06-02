#!/usr/bin/env python3
"""Example external client for shared-memory robot I/O."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from robot_shared_memory import DEFAULT_SHM_NAME, RobotSharedMemory


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read robot state and optionally write torque commands."
    )
    parser.add_argument(
        "--name",
        default=DEFAULT_SHM_NAME,
        help="Shared-memory block name.",
    )
    parser.add_argument(
        "--torque",
        type=float,
        nargs="*",
        help="Torque command in actuator order. Omit to read state only.",
    )
    parser.add_argument(
        "--disable-command",
        action="store_true",
        help="Disable external torque command and send zero torque.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to run. Use 0 for one read/write cycle.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Client loop rate in Hz when --duration is greater than 0.",
    )
    parser.add_argument(
        "--log",
        type=Path,
        help="Optional CSV file for sampled state summaries.",
    )
    return parser.parse_args()


def format_values(values: list[float], limit: int = 6) -> str:
    shown = ", ".join(f"{value:.4g}" for value in values[:limit])
    if len(values) > limit:
        shown += ", ..."
    return f"[{shown}]"


def run_cycle(shared_io: RobotSharedMemory, args: argparse.Namespace) -> str:
    state = shared_io.read_state()

    if args.disable_command:
        shared_io.disable_command()
    elif args.torque is not None:
        shared_io.write_torque(args.torque)

    return (
        f"t={state.sim_time:.4f} "
        f"alive={int(state.sim_alive)} "
        f"qpos={format_values(state.qpos)} "
        f"qvel={format_values(state.qvel)} "
        f"ctrl={format_values(state.ctrl)} "
        f"actuator_force={format_values(state.actuator_force)}"
    )


def main() -> int:
    args = parse_args()
    if args.rate <= 0:
        raise ValueError("--rate must be greater than 0.")

    with RobotSharedMemory.attach(args.name) as shared_io:
        header = shared_io.read_header()
        if args.torque is not None and len(args.torque) != header.nu:
            raise ValueError(
                f"--torque needs {header.nu} values for this model, "
                f"got {len(args.torque)}."
            )

        log_file = None
        if args.log:
            log_file = args.log.open("w", encoding="utf-8")
            log_file.write("wall_time,sim_time,sim_alive,qpos,qvel,ctrl,actuator_force\n")

        try:
            deadline = time.monotonic() + args.duration
            while True:
                summary = run_cycle(shared_io, args)
                print(summary)

                if log_file:
                    state = shared_io.read_state()
                    log_file.write(
                        f"{state.wall_time:.9f},{state.sim_time:.9f},"
                        f"{int(state.sim_alive)},"
                        f'"{state.qpos}","{state.qvel}",'
                        f'"{state.ctrl}","{state.actuator_force}"\n'
                    )
                    log_file.flush()

                if args.duration <= 0 or time.monotonic() >= deadline:
                    break
                time.sleep(1.0 / args.rate)
        finally:
            if log_file:
                log_file.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

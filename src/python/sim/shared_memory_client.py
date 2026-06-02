#!/usr/bin/env python3
"""Example external client for shared-memory robot I/O."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from robot_shared_memory import DEFAULT_CONFIG_PATH, RobotSharedMemory


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read robot state and optionally write torque commands."
    )
    parser.add_argument(
        "--name",
        help="Override the shared-memory block name from the JSON config.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="JSON config describing shared-memory state and command fields.",
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

    field_summary = " ".join(
        f"{name}={format_values(values)}" for name, values in state.fields.items()
    )
    return (
        f"t={state.sim_time:.4f} "
        f"alive={int(state.sim_alive)} "
        f"{field_summary}"
    )


def main() -> int:
    args = parse_args()
    if args.rate <= 0:
        raise ValueError("--rate must be greater than 0.")

    with RobotSharedMemory.attach(args.name, config_path=args.config) as shared_io:
        if args.torque is not None and "torque" not in shared_io.layout.command_fields:
            raise ValueError("The JSON config does not define a 'torque' command field.")
        if args.torque is not None:
            expected_torque_size = shared_io.layout.command_fields["torque"].size
        if args.torque is not None and len(args.torque) != expected_torque_size:
            raise ValueError(
                f"--torque needs {expected_torque_size} values for this config, "
                f"got {len(args.torque)}."
            )

        log_file = None
        if args.log:
            log_file = args.log.open("w", encoding="utf-8")
            field_names = list(shared_io.layout.state_fields)
            log_file.write(
                ",".join(["wall_time", "sim_time", "sim_alive", *field_names])
                + "\n"
            )

        try:
            deadline = time.monotonic() + args.duration
            while True:
                summary = run_cycle(shared_io, args)
                print(summary)

                if log_file:
                    state = shared_io.read_state()
                    field_values = [
                        f'"{state.fields[name]}"'
                        for name in shared_io.layout.state_fields
                    ]
                    log_file.write(
                        f"{state.wall_time:.9f},{state.sim_time:.9f},"
                        f"{int(state.sim_alive)},"
                        f"{','.join(field_values)}\n"
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

#!/usr/bin/env python3
"""Example external client for shared-memory robot I/O."""

from __future__ import annotations

import sys
import time
from pathlib import Path
from types import SimpleNamespace

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
if str(CONFIG_DIR) not in sys.path:
    sys.path.insert(0, str(CONFIG_DIR))

from app_config import config_section, resolve_project_path
from robot_shared_memory import RobotSharedMemory


def load_client_config() -> SimpleNamespace:
    config = config_section("sim", "shared_memory_client")
    torque = config.get("torque")
    if torque is not None:
        torque = [float(value) for value in torque]
    return SimpleNamespace(
        name=config.get("name"),
        config=resolve_project_path(config.get("config")),
        torque=torque,
        disable_command=bool(config.get("disable_command", False)),
        duration=float(config.get("duration", 0.0)),
        rate=float(config.get("rate", 50.0)),
        log=resolve_project_path(config.get("log")),
    )


def format_values(values: list[float], limit: int = 6) -> str:
    shown = ", ".join(f"{value:.4g}" for value in values[:limit])
    if len(values) > limit:
        shown += ", ..."
    return f"[{shown}]"


def run_cycle(shared_io: RobotSharedMemory, args: SimpleNamespace) -> str:
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
    args = load_client_config()
    if args.rate <= 0:
        raise ValueError("sim.shared_memory_client.rate must be greater than 0.")
    if args.config is None:
        raise ValueError("sim.shared_memory_client.config must be a path string.")
    if args.name is not None and not isinstance(args.name, str):
        raise ValueError("sim.shared_memory_client.name must be a string or null.")

    with RobotSharedMemory.attach(args.name, config_path=args.config) as shared_io:
        if args.torque is not None and "torque" not in shared_io.layout.command_fields:
            raise ValueError("The JSON config does not define a 'torque' command field.")
        if args.torque is not None:
            expected_torque_size = shared_io.layout.command_fields["torque"].size
        if args.torque is not None and len(args.torque) != expected_torque_size:
            raise ValueError(
                f"sim.shared_memory_client.torque needs {expected_torque_size} values, "
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

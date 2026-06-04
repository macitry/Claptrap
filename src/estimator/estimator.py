#!/usr/bin/env python3
"""Estimator entry point for reading the current simulator sensor state."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Sequence
import pinocchio as pin

SIM_DIR = Path(__file__).resolve().parents[1] / "sim"
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

from robot_shared_memory import DEFAULT_CONFIG_PATH, RobotSharedMemory, RobotState


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read the current sensor data from the simulator shared memory."
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
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to run. Use 0 for one sensor read.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Estimator read rate in Hz when --duration is greater than 0.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="Seconds to wait for a stable shared-memory state read.",
    )
    return parser.parse_args()


def format_values(values: Sequence[float], limit: int = 8) -> str:
    shown = ", ".join(f"{value:.6g}" for value in values[:limit])
    if len(values) > limit:
        shown += ", ..."
    return f"[{shown}]"


def format_eigenpy_example(qpos: Sequence[float], qvel: Sequence[float]) -> str:
    try:
        import eigenpy
        import numpy as np
    except ImportError as exc:
        missing_name = exc.name or "module"
        return f"eigenpy=unavailable({missing_name})"

    dimension = max(1, min(4, max(len(qpos), len(qvel))))
    q = np.zeros(dimension, dtype=float)
    v = np.zeros(dimension, dtype=float)
    q[: min(dimension, len(qpos))] = qpos[:dimension]
    v[: min(dimension, len(qvel))] = qvel[:dimension]

    # EigenPy accepts NumPy arrays and dispatches them to Eigen-backed solvers.
    sample = np.vstack((q, v))
    gram_matrix = sample.T @ sample + np.eye(dimension) * 1e-9
    eigensolver = eigenpy.SelfAdjointEigenSolver(gram_matrix)
    if eigensolver.info() != eigenpy.ComputationInfo.Success:
        return f"eigenpy=solver_info({eigensolver.info()})"

    return f"eigenpy_eigs={format_values(eigensolver.eigenvalues())}"


def read_current_sensor_data(
    shared_io: RobotSharedMemory,
    *,
    timeout: float = 1.0,
) -> tuple[RobotState, list[float]]:
    """Return the latest stable state and its MuJoCo sensordata field."""
    state = shared_io.read_state(timeout=timeout)
    try:
        return state, state.fields["sensordata"]
    except KeyError as exc:
        available = ", ".join(state.fields) or "none"
        raise KeyError(
            "The shared-memory config does not define a 'sensordata' state field. "
            f"Available state fields: {available}."
        ) from exc


def run_once(shared_io: RobotSharedMemory, timeout: float) -> str:
    state, sensordata = read_current_sensor_data(shared_io, timeout=timeout)
    qpos = state.fields.get("qpos", [])
    qvel = state.fields.get("qvel", [])
    eigenpy_example = format_eigenpy_example(qpos, qvel)

    return (
        f"t={state.sim_time:.4f} "
        f"alive={int(state.sim_alive)} "
        f"sensordata={format_values(sensordata)} "
        f"qpos={format_values(qpos)} "
        f"qvel={format_values(qvel)} "
        f"{eigenpy_example}"
    )


def main() -> int:
    args = parse_args()
    if args.rate <= 0:
        raise ValueError("--rate must be greater than 0.")
    if args.timeout <= 0:
        raise ValueError("--timeout must be greater than 0.")

    with RobotSharedMemory.attach(args.name, config_path=args.config) as shared_io:
        deadline = time.monotonic() + args.duration
        while True:
            print(run_once(shared_io, args.timeout))

            if args.duration <= 0 or time.monotonic() >= deadline:
                break
            time.sleep(1.0 / args.rate)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

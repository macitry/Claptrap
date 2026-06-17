#!/usr/bin/env python3
"""PID attitude controller using estimator output from shared memory."""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

SIM_DIR = Path(__file__).resolve().parents[1] / "sim"
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))
CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
if str(CONFIG_DIR) not in sys.path:
    sys.path.insert(0, str(CONFIG_DIR))

from app_config import config_section, resolve_project_path
from robot_shared_memory import RobotSharedMemory


@dataclass(frozen=True)
class PIDGains:
    kp: np.ndarray
    ki: np.ndarray
    kd: np.ndarray


def vector_from_config(value: object, *, default: list[float], name: str) -> np.ndarray:
    if value is None:
        value = default
    result = np.asarray(value, dtype=float)
    if result.shape != (2,):
        raise ValueError(f"controller.controller.{name} must contain 2 values.")
    return result


def index_vector_from_config(
    value: object,
    *,
    default: list[int],
    name: str,
) -> np.ndarray:
    if value is None:
        value = default
    result = np.asarray(value, dtype=int)
    if result.shape != (2,) or np.any(result < 0) or np.any(result > 1):
        raise ValueError(f"controller.controller.{name} must contain 2 indices, 0 or 1.")
    return result


def clamp_vector(values: np.ndarray, limits: np.ndarray) -> np.ndarray:
    return np.clip(values, -limits, limits)


class PIDAttitudeController:
    def __init__(
        self,
        *,
        shared_memory_name: str | None,
        shared_memory_config: Path,
        target_q: np.ndarray,
        torque_q_indices: np.ndarray,
        gains: PIDGains,
        torque_limits: np.ndarray,
        integral_limits: np.ndarray,
        loop_rate: float,
    ) -> None:
        self.shared_memory = RobotSharedMemory.attach(
            shared_memory_name,
            config_path=shared_memory_config,
        )
        self.target_q = target_q
        self.torque_q_indices = torque_q_indices
        self.gains = gains
        self.torque_limits = torque_limits
        self.integral_limits = integral_limits
        self.loop_rate = float(loop_rate)

        self.error_integral = np.zeros(2)
        self.prev_error: np.ndarray | None = None
        self.prev_time: float | None = None

    def close(self) -> None:
        self.shared_memory.close()

    def disable(self) -> None:
        self.shared_memory.disable_command()

    def step(self) -> tuple[np.ndarray, np.ndarray]:
        estimate = self.shared_memory.read_estimate()
        q_hat = np.asarray(estimate.q_hat, dtype=float)
        now = time.monotonic()

        if q_hat.shape != (2,):
            raise ValueError(f"q_hat must contain 2 values, got {q_hat.size}.")

        if self.prev_time is None:
            dt = 1.0 / self.loop_rate if self.loop_rate > 0 else 0.0
        else:
            dt = max(now - self.prev_time, 1e-6)

        q_control = q_hat[self.torque_q_indices]
        target_control = self.target_q[self.torque_q_indices]
        error = target_control - q_control
        self.error_integral += error * dt
        self.error_integral = clamp_vector(self.error_integral, self.integral_limits)

        if self.prev_error is None:
            error_derivative = np.zeros(2)
        else:
            error_derivative = (error - self.prev_error) / dt

        torque = -(
            self.gains.kp * error
            + self.gains.ki * self.error_integral
            + self.gains.kd * error_derivative
        )
        torque = clamp_vector(torque, self.torque_limits)
        self.shared_memory.write_torque(torque)

        self.prev_error = error
        self.prev_time = now
        return q_hat, torque

    def run(self) -> None:
        if self.loop_rate <= 0:
            raise ValueError("controller.controller.loop_rate must be greater than 0.")

        while True:
            q_hat, torque = self.step()
            print(f"q_hat={q_hat} torque={torque}")
            time.sleep(1.0 / self.loop_rate)


def main() -> int:
    config = config_section("controller", "controller")
    shared_memory_name = config.get("shared_memory_name")
    shared_memory_config = resolve_project_path(config.get("shared_memory_config"))
    if shared_memory_name is not None and not isinstance(shared_memory_name, str):
        raise ValueError("controller.controller.shared_memory_name must be a string or null.")
    if shared_memory_config is None:
        raise ValueError("controller.controller.shared_memory_config must be a path string.")

    target_q = vector_from_config(config.get("target_q"), default=[0.0, 0.0], name="target_q")
    torque_q_indices = index_vector_from_config(
        config.get("torque_q_indices"),
        default=[1, 0],
        name="torque_q_indices",
    )
    gains = PIDGains(
        kp=vector_from_config(config.get("kp"), default=[10.0, 10.0], name="kp"),
        ki=vector_from_config(config.get("ki"), default=[0.0, 0.0], name="ki"),
        kd=vector_from_config(config.get("kd"), default=[1.0, 1.0], name="kd"),
    )
    torque_limits = vector_from_config(
        config.get("torque_limits"),
        default=[3.0, 3.0],
        name="torque_limits",
    )
    integral_limits = vector_from_config(
        config.get("integral_limits"),
        default=[0.5, 0.5],
        name="integral_limits",
    )
    loop_rate = float(config.get("loop_rate", 50.0))

    try:
        controller = PIDAttitudeController(
            shared_memory_name=shared_memory_name,
            shared_memory_config=shared_memory_config,
            target_q=target_q,
            torque_q_indices=torque_q_indices,
            gains=gains,
            torque_limits=torque_limits,
            integral_limits=integral_limits,
            loop_rate=loop_rate,
        )
    except FileNotFoundError as exc:
        raise SystemExit(
            "Shared memory is not available. Start the simulator first with:\n"
            "  uv run python src/sim/launch_robot_scene.py\n"
            "Then start the estimator with:\n"
            "  uv run python src/estimator/estimator.py"
        ) from exc

    try:
        controller.run()
    except KeyboardInterrupt:
        return 0
    finally:
        controller.disable()
        controller.close()


if __name__ == "__main__":
    raise SystemExit(main())

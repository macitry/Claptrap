#!/usr/bin/env python3
"""Estimator entry point for reading the current simulator sensor state."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Sequence
import pinocchio as pin
import eigenpy
import numpy as np
SIM_DIR = Path(__file__).resolve().parents[1] / "sim"
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

from robot_shared_memory import DEFAULT_CONFIG_PATH, RobotSharedMemory, RobotState


class IMU:
    w: np.ndarray
    a: np.ndarray


def rotation_x(angle: float) -> np.ndarray:
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cos_angle, -sin_angle],
            [0.0, sin_angle, cos_angle],
        ]
    )


def rotation_y(angle: float) -> np.ndarray:
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    return np.array(
        [
            [cos_angle, 0.0, sin_angle],
            [0.0, 1.0, 0.0],
            [-sin_angle, 0.0, cos_angle],
        ]
    )


def attitude_rate_jacobian(q1_hat: float, q2_hat: float) -> np.ndarray:
    """Return J(q) for omega_base = J(q) @ qdot_G."""
    e1 = np.array([1.0, 0.0, 0.0])
    e2 = np.array([0.0, 1.0, 0.0])
    e3 = np.array([0.0, 0.0, 1.0])

    r1 = rotation_x(q1_hat)
    r2 = rotation_y(q2_hat)
    return np.column_stack((r2.T @ e1, e2, r2.T @ r1.T @ e3))


def generalized_attitude_rate(
    imu_w_avg_base: np.ndarray,
    q1_hat: float,
    q2_hat: float,
) -> np.ndarray:
    jacobian = attitude_rate_jacobian(q1_hat, q2_hat)
    return np.linalg.solve(jacobian, imu_w_avg_base)


class Estimator:
    def __init__(self, config_path: str = DEFAULT_CONFIG_PATH):
        self.shared_memory = RobotSharedMemory(config_path)

        self.imu1 = IMU()
        self.imu1.w = np.zeros(3)
        self.imu2 = IMU()
        self.imu2.w = np.zeros(3)
        self.imu3 = IMU()
        self.imu3.w = np.zeros(3)
        self.imu4 = IMU()
        self.imu4.w = np.zeros(3)

        self.q_prev = np.zeros(3)

        self.dt = 0.01
        self.q1_hat = 0.0
        self.q2_hat = 0.0

    def read_state(self) -> RobotState:
        return self.shared_memory.read_state()

    def estimate(self, state: RobotState) -> None:

        while True:
            state = self.read_state()
            print(f"state={state}")

            # 基于四个IMU的平均角速度进行估计机体角度。
            imu_w_avg_imu = np.average(
                [self.imu1.w, self.imu2.w, self.imu3.w, self.imu4.w], axis=0
            )

            base_R_imu = np.eye(3)
            imu_w_avg_base = base_R_imu @ imu_w_avg_imu


            qdot_g = generalized_attitude_rate(
                imu_w_avg_base,
                self.q1_hat,
                self.q2_hat,
            )

            print(f"imu_w_avg_base={imu_w_avg_base}")
            print(f"J={attitude_rate_jacobian(self.q1_hat, self.q2_hat)}")
            print(f"qdot_G={qdot_g}")

            self.q_prev = self.q_prev + qdot_g * self.dt



def main() -> int:









    return 0


if __name__ == "__main__":
    raise SystemExit(main())

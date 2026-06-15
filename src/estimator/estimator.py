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
    w: np.ndarray  # 角速度
    a: np.ndarray  # 线加速度
    B_p_w: np.ndarray  # IMU到车轮重心的位置向量,相对于base坐标系


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
        self.q1_gy_hat = 0.0
        self.q2_gy_hat = 0.0

        self.w_heels = np.zeros(3)

        self.imu1.B_p_w = np.zeros(3)
        self.imu2.B_p_w = np.zeros(3)
        self.imu3.B_p_w = np.zeros(3)
        self.imu4.B_p_w = np.zeros(3)

        self.P = np.array([[1.0, self.imu1.B_p_w],
                           [1.0, self.imu2.B_p_w],
                           [1.0, self.imu3.B_p_w],
                           [1.0, self.imu4.B_p_w]])

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

            self.q_gy_prev = self.q_gy_prev + qdot_g * self.dt
            self.q1_gy_hat = self.q_gy_prev[0]
            self.q2_gy_hat = self.q_gy_prev[1]

        # 基于加速度计的重力方向估计机体角度.

            # 1.计算车轮加速度
            ddq_wheels = self.r * self.w_heels

            base_ddq_wheels = ddq_wheels #假设姿态保持在较小的范围内，旋转矩阵近似为单位矩阵

            M_hat=np.array([[self.imu1.a-base_ddq_wheels],
                           [self.imu2.a-base_ddq_wheels],
                           [self.imu3.a-base_ddq_wheels],
                           [self.imu4.a-base_ddq_wheels]]
            )
            P_0=self.P[:,0]
            B_g=-self.M*P_0

            B_g1_hat, B_g2_hat, B_g3_hat = B_g

            q1_A_hat = np.arctan2(
                B_g2_hat,
                np.sqrt(B_g1_hat**2 + B_g3_hat**2),
            )
            q2_A_hat = np.arctan2(-B_g1_hat, B_g3_hat)

        #   一阶互补滤波融合陀螺仪和加速度计的估计
            self.alpha = 0.5
            q = self.alpha*np.array([q1_A_hat, q2_A_hat])+(1-self.alpha)*np.array([self.q1_gy_hat, self.q2_gy_hat])
            print(f"q={q}")













def main() -> int:









    return 0


if __name__ == "__main__":
    raise SystemExit(main())

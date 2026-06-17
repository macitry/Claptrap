#!/usr/bin/env python3
"""Estimator entry point for reading the current simulator sensor state."""

from __future__ import annotations

import sys
import time
from pathlib import Path
import pinocchio as pin
import eigenpy
import numpy as np
SIM_DIR = Path(__file__).resolve().parents[1] / "sim"
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))
CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
if str(CONFIG_DIR) not in sys.path:
    sys.path.insert(0, str(CONFIG_DIR))

from app_config import config_section, resolve_project_path
from robot_shared_memory import RobotSharedMemory, RobotState


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


def accelerometer_attitude_from_gravity(B_g_hat: np.ndarray) -> tuple[float, float]:
    B_g_hat = np.asarray(B_g_hat, dtype=float).reshape(3)
    B_g1_hat, B_g2_hat, B_g3_hat = B_g_hat

    q1_A_hat = np.arctan2(
        B_g2_hat,
        np.sqrt(B_g1_hat**2 + B_g3_hat**2),
    )
    q2_A_hat = np.arctan2(-B_g1_hat, B_g3_hat)

    return q1_A_hat, q2_A_hat


def imu_positions_from_config(value: object | None) -> np.ndarray:
    if value is None:
        return np.zeros((4, 3))

    positions = np.asarray(value, dtype=float)
    if positions.shape != (4, 3):
        raise ValueError(
            "estimator.estimator.imu_positions_in_base must be a 4x3 list."
        )
    return positions


class Estimator:
    def __init__(
        self,
        *,
        shared_memory_name: str | None = None,
        shared_memory_config: Path,
        wheel_radius: float = 0.0,
        imu_positions_in_base: object | None = None,
        alpha: float = 0.5,
        loop_rate: float = 50.0,
    ):
        self.shared_memory = RobotSharedMemory.attach(
            shared_memory_name,
            config_path=shared_memory_config,
        )

        self.imu1 = IMU()
        self.imu1.w = np.zeros(3)
        self.imu1.a = np.zeros(3)
        self.imu2 = IMU()
        self.imu2.w = np.zeros(3)
        self.imu2.a = np.zeros(3)
        self.imu3 = IMU()
        self.imu3.w = np.zeros(3)
        self.imu3.a = np.zeros(3)
        self.imu4 = IMU()
        self.imu4.w = np.zeros(3)
        self.imu4.a = np.zeros(3)
        self.imus = [self.imu1, self.imu2, self.imu3, self.imu4]

        self.q_prev = np.zeros(3)
        self.q_gy_prev = np.zeros(3)

        self.dt = 0.01
        self.q1_gy_hat = 0.0
        self.q2_gy_hat = 0.0
        self.q1_A_hat = 0.0
        self.q2_A_hat = 0.0
        self.alpha = float(alpha)
        self.loop_rate = float(loop_rate)

        self.w_wheels = np.zeros(3)
        self.r = float(wheel_radius)

        imu_positions = imu_positions_from_config(imu_positions_in_base)
        for imu, position in zip(self.imus, imu_positions):
            imu.B_p_w = position

        self.P = np.vstack(
            [np.r_[1.0, imu.B_p_w] for imu in self.imus]
        )
        self.P_pinv = np.linalg.pinv(self.P)

    def close(self) -> None:
        self.shared_memory.close()

    def read_state(self) -> RobotState:
        return self.shared_memory.read_state()

    def update_imus_from_state(self, state: RobotState) -> None:
        sensordata = np.asarray(state.sensordata, dtype=float)
        values_per_imu = 10  # framequat(4) + gyro(3) + accelerometer(3)
        expected_size = len(self.imus) * values_per_imu
        if sensordata.size < expected_size:
            return

        for index, imu in enumerate(self.imus):
            offset = index * values_per_imu
            imu.w = sensordata[offset + 4: offset + 7]
            imu.a = sensordata[offset + 7: offset + 10]

    def estimate(self, state: RobotState | None = None) -> None:
        if state is not None:
            self.update_imus_from_state(state)

        while True:
            state = self.read_state()
            self.update_imus_from_state(state)
            self.dt = state.timestep
            print(f"state={state}")

        #   基于四个IMU的平均角速度进行估计机体角度。
            imu_w_avg_imu = np.average(
                [self.imu1.w, self.imu2.w, self.imu3.w, self.imu4.w], axis=0
            )

            base_R_imu = np.eye(3)
            imu_w_avg_base = base_R_imu @ imu_w_avg_imu

            qdot_g = generalized_attitude_rate(
                imu_w_avg_base,
                self.q1_gy_hat,
                self.q2_gy_hat,
            )

            print(f"imu_w_avg_base={imu_w_avg_base}")
            print(f"J={attitude_rate_jacobian(self.q1_gy_hat, self.q2_gy_hat)}")
            print(f"qdot_G={qdot_g}")

            self.q_gy_prev = self.q_gy_prev + qdot_g * self.dt
            self.q1_gy_hat = self.q_gy_prev[0]
            self.q2_gy_hat = self.q_gy_prev[1]
            print(f"q_gy_prev=({self.q_gy_prev})") # TODO:注意此处的计算的方向

        #   基于加速度计的重力方向估计机体角度.

            # 1.计算车轮加速度
            ddq_wheels = self.r * self.w_wheels

            base_ddq_wheels = ddq_wheels  # 假设姿态保持在较小的范围内，旋转矩阵近似为单位矩阵

            M_hat = np.vstack(
                [
                    self.imu1.a - base_ddq_wheels,
                    self.imu2.a - base_ddq_wheels,
                    self.imu3.a - base_ddq_wheels,
                    self.imu4.a - base_ddq_wheels,
                ]
            )
            M = self.P_pinv @ M_hat
            B_g_hat = -M[0]
            self.q1_A_hat, self.q2_A_hat = accelerometer_attitude_from_gravity(B_g_hat)
            print(f"q1_A_hat={self.q1_A_hat}, q2_A_hat={self.q2_A_hat}")
        #   一阶互补滤波融合陀螺仪和加速度计的估计
            q_hat = self.alpha * np.array([self.q1_A_hat, self.q2_A_hat]) + (
                1 - self.alpha
            ) * np.array([self.q1_gy_hat, self.q2_gy_hat])
            self.shared_memory.write_estimate(q_hat=q_hat)
            print(f"q_hat={q_hat}")

            if self.loop_rate > 0:
                time.sleep(1.0 / self.loop_rate)
    

def main() -> int:
    config = config_section("estimator", "estimator")
    shared_memory_name = config.get("shared_memory_name")
    shared_memory_config = resolve_project_path(config.get("shared_memory_config"))
    wheel_radius = float(config.get("wheel_radius", 0.0))
    imu_positions_in_base = config.get("imu_positions_in_base")
    alpha = float(config.get("alpha", 0.5))
    loop_rate = float(config.get("loop_rate", 50.0))
    if shared_memory_name is not None and not isinstance(shared_memory_name, str):
        raise ValueError("estimator.estimator.shared_memory_name must be a string or null.")
    if shared_memory_config is None:
        raise ValueError("estimator.estimator.shared_memory_config must be a path string.")

    try:
        estimator = Estimator(
            shared_memory_name=shared_memory_name,
            shared_memory_config=shared_memory_config,
            wheel_radius=wheel_radius,
            imu_positions_in_base=imu_positions_in_base,
            alpha=alpha,
            loop_rate=loop_rate,
        )
    except FileNotFoundError as exc:
        raise SystemExit(
            "Shared memory is not available. Start the simulator first with:\n"
            "  uv run python src/sim/launch_robot_scene.py"
        ) from exc

    try:
        estimator.estimate(estimator.read_state())
    except KeyboardInterrupt:
        return 0
    finally:
        estimator.close()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())

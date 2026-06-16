#!/usr/bin/env python3
"""Small Pinocchio example for estimator-side model computations."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
if str(CONFIG_DIR) not in sys.path:
    sys.path.insert(0, str(CONFIG_DIR))

from app_config import config_section, require_project_path


def project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def load_example_config() -> SimpleNamespace:
    config = config_section("estimator", "pinocchio_example")
    imu_angular_velocity = config.get("imu_angular_velocity", [0.10, -0.20, 0.30])
    body_rpy = config.get("body_rpy", [0.0, 0.0, 0.0])
    if len(imu_angular_velocity) != 3:
        raise ValueError(
            "estimator.pinocchio_example.imu_angular_velocity must have 3 values."
        )
    if len(body_rpy) != 3:
        raise ValueError("estimator.pinocchio_example.body_rpy must have 3 values.")

    imu_frame = config.get("imu_frame")
    if imu_frame is not None and not isinstance(imu_frame, str):
        raise ValueError("estimator.pinocchio_example.imu_frame must be a string or null.")

    return SimpleNamespace(
        urdf=require_project_path(config, "urdf"),
        floating_base=bool(config.get("floating_base", False)),
        imu_frame=imu_frame,
        imu_angular_velocity=[float(value) for value in imu_angular_velocity],
        body_rpy=[float(value) for value in body_rpy],
    )


def import_pinocchio():
    try:
        import pinocchio as pin
    except ImportError as exc:
        raise SystemExit(
            "Missing Pinocchio. Run this with `uv run python "
            "src/estimator/pinocchio_example.py` or install the `pin` package."
        ) from exc

    if not hasattr(pin, "buildModelFromUrdf"):
        module_path = getattr(pin, "__file__", "<namespace package>")
        raise SystemExit(
            "Imported a module named `pinocchio`, but it is not the Pinocchio "
            f"robotics bindings: {module_path}. Run through `uv run` so the "
            "installed `pin` package is used."
        )

    return pin


def build_model(pin, urdf_path: Path, floating_base: bool):
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF does not exist: {urdf_path}")

    if floating_base:
        return pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    return pin.buildModelFromUrdf(str(urdf_path))


def set_joint_vector_entry(model, values: np.ndarray, joint_name: str, value: float) -> None:
    joint_id = model.getJointId(joint_name)
    if joint_id >= model.njoints:
        return

    velocity_index = model.joints[joint_id].idx_v
    if velocity_index < values.shape[0]:
        values[velocity_index] = value


def format_vector(values: np.ndarray, precision: int = 6) -> str:
    return "[" + ", ".join(f"{float(value):.{precision}g}" for value in values) + "]"


def format_matrix(matrix: np.ndarray, precision: int = 6) -> str:
    return np.array2string(matrix, precision=precision, suppress_small=True)


def frame_translation(model, data, frame_name: str) -> str:
    frame_id = model.getFrameId(frame_name)
    if frame_id >= model.nframes:
        return "missing"
    return format_vector(data.oMf[frame_id].translation)


def frame_names_with_prefix(model, prefix: str) -> list[str]:
    return [frame.name for frame in model.frames if frame.name.startswith(prefix)]


def relative_homogeneous_matrix(model, data, parent_frame: str, child_frame: str) -> np.ndarray:
    parent_id = model.getFrameId(parent_frame)
    child_id = model.getFrameId(child_frame)
    if parent_id >= model.nframes:
        raise ValueError(f"Unknown parent frame: {parent_frame}")
    if child_id >= model.nframes:
        raise ValueError(f"Unknown child frame: {child_frame}")

    parent_placement = data.oMf[parent_id]
    child_placement = data.oMf[child_id]
    parent_to_child = parent_placement.inverse() * child_placement
    return np.asarray(parent_to_child.homogeneous)


def angular_velocity_in_world(
    model,
    data,
    imu_frame: str,
    angular_velocity_imu: np.ndarray,
) -> np.ndarray:
    imu_id = model.getFrameId(imu_frame)
    if imu_id >= model.nframes:
        raise ValueError(f"Unknown IMU frame: {imu_frame}")

    world_rotation_imu = data.oMf[imu_id].rotation
    return world_rotation_imu @ angular_velocity_imu


def angular_velocity_between_frames(
    model,
    data,
    target_frame: str,
    source_frame: str,
    angular_velocity_source: np.ndarray,
) -> np.ndarray:
    target_id = model.getFrameId(target_frame)
    source_id = model.getFrameId(source_frame)
    if target_id >= model.nframes:
        raise ValueError(f"Unknown target frame: {target_frame}")
    if source_id >= model.nframes:
        raise ValueError(f"Unknown source frame: {source_frame}")

    world_rotation_target = data.oMf[target_id].rotation
    world_rotation_source = data.oMf[source_id].rotation
    target_rotation_source = world_rotation_target.T @ world_rotation_source
    return target_rotation_source @ angular_velocity_source


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


def euler_rate_matrix(q1_g: float, q2_g: float) -> np.ndarray:
    e1 = np.array([1.0, 0.0, 0.0])
    e2 = np.array([0.0, 1.0, 0.0])
    e3 = np.array([0.0, 0.0, 1.0])
    r1 = rotation_x(q1_g)
    r2 = rotation_y(q2_g)
    return np.column_stack((r2.T @ e1, e2, r2.T @ r1.T @ e3))


def generalized_rates_from_body_omega(
    body_omega: np.ndarray,
    q1_g: float,
    q2_g: float,
) -> np.ndarray:
    return np.linalg.solve(euler_rate_matrix(q1_g, q2_g), body_omega)


def main() -> int:
    args = load_example_config()
    pin = import_pinocchio()

    model = build_model(pin, args.urdf, args.floating_base)
    data = model.createData()

    q0 = pin.neutral(model)
    dq = np.zeros(model.nv)
    set_joint_vector_entry(model, dq, "base_link_to_link1", 0.20)
    set_joint_vector_entry(model, dq, "plane1_to_link2", -0.10)
    q = pin.integrate(model, q0, dq)

    v = np.zeros(model.nv)
    set_joint_vector_entry(model, v, "base_link_to_link1", 0.50)
    set_joint_vector_entry(model, v, "plane1_to_link2", -0.25)
    a = np.zeros(model.nv)

    pin.forwardKinematics(model, data, q, v, a)
    pin.updateFramePlacements(model, data)
    center_of_mass = pin.centerOfMass(model, data, q)
    total_mass = pin.computeTotalMass(model)
    mass_matrix = np.asarray(pin.crba(model, data, q))
    tau = pin.rnea(model, data, q, v, a)
    imu_angular_velocity = np.asarray(args.imu_angular_velocity, dtype=float)
    imu_frames = [args.imu_frame] if args.imu_frame else frame_names_with_prefix(model, "imu_")
    body_rpy = np.asarray(args.body_rpy, dtype=float)

    print(f"pinocchio version: {getattr(pin, '__version__', 'unknown')}")
    print(f"model: {model.name}")
    print(f"nq={model.nq} nv={model.nv} njoints={model.njoints} nframes={model.nframes}")
    print(f"joint names: {', '.join(model.names.tolist())}")
    print(f"total mass: {total_mass:.6g}")
    print(f"center of mass: {format_vector(center_of_mass)}")
    print(f"mass matrix diagonal: {format_vector(np.diag(mass_matrix))}")
    print(f"rnea torque: {format_vector(tau)}")
    print(f"base_link position: {frame_translation(model, data, 'base_link')}")
    print(f"link1 position: {frame_translation(model, data, 'link1')}")
    print(f"link2 position: {frame_translation(model, data, 'link2')}")
    print(f"imu angular velocity in imu frame: {format_vector(imu_angular_velocity)}")
    print(f"body rpy for Euler-rate inverse: {format_vector(body_rpy)}")

    body_omegas = []
    for imu_frame in imu_frames:
        base_to_imu = relative_homogeneous_matrix(model, data, "base_link", imu_frame)
        omega_base_link = angular_velocity_between_frames(
            model,
            data,
            "base_link",
            imu_frame,
            imu_angular_velocity,
        )
        body_omegas.append(omega_base_link)
        # The IMU is fixed to the body, so this is the body angular velocity
        # expressed in the inertial/world frame.
        omega_world = angular_velocity_in_world(
            model,
            data,
            imu_frame,
            imu_angular_velocity,
        )
        print(f"base_link_H_{imu_frame}:")
        print(format_matrix(base_to_imu))
        print(f"body_omega_base_link_from_{imu_frame}: {format_vector(omega_base_link)}")
        print(f"body_omega_world_from_{imu_frame}: {format_vector(omega_world)}")

    body_omega_average = np.mean(body_omegas, axis=0)
    qdot_g = generalized_rates_from_body_omega(
        body_omega_average,
        body_rpy[0],
        body_rpy[1],
    )
    print("euler_rate_matrix:")
    print(format_matrix(euler_rate_matrix(body_rpy[0], body_rpy[1])))
    print(f"body_omega_base_link_average: {format_vector(body_omega_average)}")
    print(f"qdot_G_from_average_imu: {format_vector(qdot_g)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

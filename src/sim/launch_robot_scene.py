#!/usr/bin/env python3
"""Launch a robot MuJoCo scene and expose robot I/O through shared memory."""

from __future__ import annotations

import shutil
import subprocess
import sys
import time
from pathlib import Path

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
if str(CONFIG_DIR) not in sys.path:
    sys.path.insert(0, str(CONFIG_DIR))

from app_config import config_section, require_project_path, resolve_project_path
from robot_shared_memory import (
    COMMAND_MODE_TORQUE,
    RobotSharedMemory,
)


def project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def default_simulate_binary() -> Path:
    return project_root() / "mujoco" / "build" / "bin" / "simulate"


def actuator_names(mujoco, model) -> list[str]:
    return [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        or f"actuator_{i}"
        for i in range(model.nu)
    ]


def joint_names(mujoco, model) -> list[str]:
    return [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        or f"joint_{i}"
        for i in range(model.njnt)
    ]


def state_field_values(shared_io: RobotSharedMemory, model, data) -> dict[str, object]:
    values: dict[str, object] = {}
    for name in shared_io.layout.state_fields:
        if hasattr(data, name):
            values[name] = getattr(data, name)
        elif hasattr(model, name):
            values[name] = getattr(model, name)
        else:
            raise ValueError(
                f"No MuJoCo data source for configured state field {name!r}."
            )
    return values


def write_sim_state(shared_io: RobotSharedMemory, model, data) -> None:
    shared_io.write_state(
        sim_time=data.time,
        timestep=model.opt.timestep,
        **state_field_values(shared_io, model, data),
    )


def clip_torque_to_ctrlrange(model, torque: list[float]) -> list[float]:
    result = list(torque)
    for i, value in enumerate(result):
        if i < model.nu and model.actuator_ctrllimited[i]:
            low, high = model.actuator_ctrlrange[i]
            result[i] = min(max(value, low), high)
    return result


def apply_external_command(shared_io: RobotSharedMemory, model, data) -> None:
    try:
        command = shared_io.read_command(timeout=0.001)
    except TimeoutError:
        return

    if command.enabled and command.mode == COMMAND_MODE_TORQUE:
        data.ctrl[:] = clip_torque_to_ctrlrange(model, command.torque)
    else:
        data.ctrl[:] = 0.0


def launch_with_python_viewer(
    scene_path: Path,
    duration: float,
    *,
    shm_name: str | None,
    shm_config: Path,
    no_unlink: bool,
) -> bool:
    """Try the MuJoCo Python passive viewer if the bindings are installed."""
    try:
        import mujoco
        import mujoco.viewer
    except Exception:
        return False

    if not hasattr(mujoco, "MjModel") or not hasattr(mujoco.viewer, "launch_passive"):
        return False

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)

    with RobotSharedMemory.create(
        name=shm_name,
        nq=model.nq,
        nv=model.nv,
        nu=model.nu,
        nsensordata=model.nsensordata,
        timestep=model.opt.timestep,
        config_path=shm_config,
        unlink_on_close=not no_unlink,
    ) as shared_io:
        print(f"Shared memory: {shared_io.name}")
        print(f"Joints: {', '.join(joint_names(mujoco, model))}")
        print(f"Torque command order: {', '.join(actuator_names(mujoco, model))}")

        write_sim_state(shared_io, model, data)

        deadline = time.monotonic() + duration
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and time.monotonic() < deadline:
                step_start = time.monotonic()
                apply_external_command(shared_io, model, data)
                mujoco.mj_step(model, data)
                write_sim_state(shared_io, model, data)
                viewer.sync()

                sleep_time = model.opt.timestep - (time.monotonic() - step_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)

    return True


def launch_with_simulate_binary(scene_path: Path, duration: float) -> None:
    """Launch the local MuJoCo simulate GUI and close it after duration."""
    print(
        "MuJoCo Python viewer is not available; falling back to the simulate "
        "binary without shared-memory robot I/O.",
        file=sys.stderr,
    )
    simulate = default_simulate_binary()
    if not simulate.exists():
        discovered = shutil.which("simulate")
        if discovered:
            simulate = Path(discovered)
        else:
            raise FileNotFoundError(
                "Could not find MuJoCo Python bindings or the simulate binary. "
                f"Expected local binary at: {simulate}"
            )

    process = subprocess.Popen([str(simulate), str(scene_path)])
    try:
        process.wait(timeout=duration)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main() -> int:
    config = config_section("sim", "launch_robot_scene")
    scene_path = require_project_path(config, "scene")
    duration = float(config.get("duration", 0.0))
    force_simulate = bool(config.get("force_simulate", False))
    shm_name = config.get("shm_name")
    shm_config = resolve_project_path(config.get("shm_config"))
    no_unlink = bool(config.get("no_unlink", False))

    if not scene_path.exists():
        print(f"Scene XML does not exist: {scene_path}", file=sys.stderr)
        return 1

    if duration <= 0:
        print("sim.launch_robot_scene.duration must be greater than 0.", file=sys.stderr)
        return 1

    if shm_name is not None and not isinstance(shm_name, str):
        print("sim.launch_robot_scene.shm_name must be a string or null.", file=sys.stderr)
        return 1

    if shm_config is None:
        print("sim.launch_robot_scene.shm_config must be a path string.", file=sys.stderr)
        return 1

    print(f"Loading scene: {scene_path}")
    print(f"Viewer will run for {duration:g} seconds.")

    if force_simulate:
        launch_with_simulate_binary(scene_path, duration)
        return 0

    if launch_with_python_viewer(
        scene_path,
        duration,
        shm_name=shm_name,
        shm_config=shm_config,
        no_unlink=no_unlink,
    ):
        return 0

    print(
        "MuJoCo Python viewer is required for shared-memory robot I/O. "
        "Install/use a Python environment with mujoco and numpy, or set "
        "sim.launch_robot_scene.force_simulate to true for display-only mode.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

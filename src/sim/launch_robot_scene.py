#!/usr/bin/env python3
"""Launch a robot MuJoCo scene and expose robot I/O through shared memory."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

from robot_shared_memory import (
    COMMAND_MODE_TORQUE,
    DEFAULT_CONFIG_PATH,
    RobotSharedMemory,
)


DEFAULT_DURATION_SECONDS = 50.0
SCENE_ENV_VAR = "ROBOT_SCENE_XML"


def project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def default_scene_path() -> Path | None:
    scene_path = os.environ.get(SCENE_ENV_VAR)
    if scene_path:
        return Path(scene_path)
    return None


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


def parse_args() -> argparse.Namespace:
    scene_default = default_scene_path()
    parser = argparse.ArgumentParser(
        description="Open a robot MJCF scene in the MuJoCo interactive viewer."
    )
    parser.add_argument(
        "--scene",
        type=Path,
        default=scene_default,
        required=scene_default is None,
        help=f"MJCF scene path to load. Can also be set with {SCENE_ENV_VAR}.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_SECONDS,
        help="How many seconds to keep the viewer running.",
    )
    parser.add_argument(
        "--force-simulate",
        action="store_true",
        help="Use the local simulate binary without shared-memory robot I/O.",
    )
    parser.add_argument(
        "--shm-name",
        help="Override the shared-memory block name from the JSON config.",
    )
    parser.add_argument(
        "--shm-config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="JSON config describing shared-memory state and command fields.",
    )
    parser.add_argument(
        "--no-unlink",
        action="store_true",
        help="Leave the shared-memory block after exit for debugging.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    scene_path = args.scene.expanduser().resolve()

    if not scene_path.exists():
        print(f"Scene XML does not exist: {scene_path}", file=sys.stderr)
        return 1

    if args.duration <= 0:
        print("--duration must be greater than 0.", file=sys.stderr)
        return 1

    print(f"Loading scene: {scene_path}")
    print(f"Viewer will run for {args.duration:g} seconds.")

    if args.force_simulate:
        launch_with_simulate_binary(scene_path, args.duration)
        return 0

    if launch_with_python_viewer(
        scene_path,
        args.duration,
        shm_name=args.shm_name,
        shm_config=args.shm_config,
        no_unlink=args.no_unlink,
    ):
        return 0

    print(
        "MuJoCo Python viewer is required for shared-memory robot I/O. "
        "Install/use a Python environment with mujoco and numpy, or run with "
        "--force-simulate for display-only mode.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

# Claptrap

Robot model and MuJoCo tooling for the Claptrap project.

## Layout

- `model/urdf/`: URDF source models
- `model/xml/`: MJCF files generated from URDF plus MuJoCo overlays and scenes
- `src/sim/`: MuJoCo simulator and shared-memory robot I/O
- `src/controller/`: controller entry point placeholder
- `src/estimator/`: estimator entry point placeholder
- `tools/`: local utilities, including the URDF-to-MJCF converter source and build script
- `mujoco/`: MuJoCo source tree pinned as a Git submodule

## Convert URDF to MJCF

```bash
./tools/build_urdf2mjcf.sh
./tools/update_mjcf.sh
```

Load the basic scene from:

```bash
model/xml/claptrap_scene.xml
```

`model/xml/claptrap.xml` is the fixed-base robot entry. `model/xml/claptrap_floating.xml`
adds a free root joint, and `model/xml/claptrap_scene.xml` uses that floating entry so
gravity can move the robot.

## Simulation

Python dependencies are managed with `uv` in the project root.

```bash
uv sync
uv run python src/sim/launch_robot_scene.py --scene model/xml/claptrap_scene.xml
```

External shared-memory clients should also use `uv run`:

```bash
uv run python src/sim/shared_memory_client.py
```

The shared-memory field layout is configured in:

```bash
src/sim/robot_shared_memory_config.json
```

Each field uses `float64` data. `size` can be a fixed integer or one of the
model dimensions: `nq`, `nv`, `nu`, `nsensordata`. Configured state fields
should match MuJoCo data exposed by the selected simulator branch.

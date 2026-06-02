# Claptrap

Robot model and MuJoCo tooling for the Claptrap project.

## Layout

- `model/urdf/`: URDF source models
- `model/xml/`: MJCF files generated from URDF plus MuJoCo overlays and scenes
- `src/config/`: runtime and shared-memory configuration
- `src/cpp/`: C++ runtime plus simulator branches
- `src/python/`: Python simulator branch
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

## C++ runtime plugins

The runtime plugin order is configured in:

```bash
src/config/robot_runtime_config.json
```

The initial built-in plugin types are `sim`, `estimator`, and `controller`.
They are configured and started in the same order as the `plugins` array, then
stopped in reverse order.

```bash
./src/cpp/runtime/build_runtime.sh
./src/cpp/runtime/robot_runtime --duration 0
```

## Python simulation environment

Python dependencies are managed with `uv` in the project root.

```bash
uv sync
uv run python src/python/sim/launch_robot_scene.py --scene model/xml/claptrap_scene.xml
```

External shared-memory clients should also use `uv run`:

```bash
uv run python src/python/sim/shared_memory_client.py
```

## C++ simulation branch

The C++ branch uses the same JSON shared-memory layout as the Python branch.
It runs the MuJoCo simulation loop headlessly and exposes the same robot I/O.

```bash
./src/cpp/sim/build_sim.sh
./src/cpp/sim/launch_robot_scene_cpp --scene model/xml/claptrap_scene.xml
./src/cpp/sim/shared_memory_client_cpp
```

The shared-memory field layout is configured in:

```bash
src/config/robot_shared_memory_config.json
```

Each field uses `float64` data. `size` can be a fixed integer or one of the
model dimensions: `nq`, `nv`, `nu`, `nsensordata`. Configured state fields
should match MuJoCo data exposed by the selected simulator branch.

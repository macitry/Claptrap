# Claptrap

Robot model and MuJoCo tooling for the Claptrap project.

## Layout

- `model/`: project robot models, including `claptrap.urdf`
- `tools/`: local utilities, including the URDF-to-MJCF converter source and build script
- `mujoco/`: MuJoCo source tree pinned as a Git submodule

## Convert URDF to MJCF

```bash
./tools/build_urdf2mjcf.sh
LD_LIBRARY_PATH="$PWD/mujoco/build/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  ./tools/urdf2mjcf model/claptrap.urdf model/claptrap.xml
```

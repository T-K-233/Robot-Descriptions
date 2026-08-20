# Lite Description

[![Python](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/)
[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)](LICENSE)

Robot description for the Berkeley Humanoid Lite V2, an open-source humanoid built on
[Robstride](https://robstride.com) actuators. This repository is the single source of
truth for the robot's geometry. It holds the URDF, MJCF and xacro descriptions and the
meshes, all generated from the Onshape CAD.

<!-- TODO: add a render / photo of the Lite robot here. -->

The same source serves both worlds:

- Simulation and RL, through the `robot_assets` Python loader, for Mujoco Lab and Isaac Lab.
- ROS 2, through `ros2_control` and `robot_state_publisher`, as the `lite_description`
  ament package.

## Variants

| Variant | Description | DoF | Root | `ros2_control` |
|---|---|---|---|---|
| `lite` | full body (legs, 1-DoF waist yaw, arms, neck, 5-finger hands) | 72 | `pelvis` | model-only |
| `lite_pro` | full body **Pro** (legs, 3-DoF waist, arms, neck, 5-finger hands) | 74 | `pelvis` | model-only |
| `lite_dummy` | V1 bimanual upper body (arms + neck), the configuration `Humanoid Control` deploys | 17 | `chest` | Robstride on two CAN buses |
| `lite_bimanual` | V2 bimanual arms, no neck | 14 | `chest` | Robstride on two CAN buses |
| `lite_biped` | V2 legs (hip x3, knee, ankle x3 per leg), floating base | 14 | `pelvis` | Robstride on two CAN buses + base IMU |
| `lite_biped_debug` | `lite_biped` with the flat debug foot instead of the rockered sole | 14 | `pelvis` | Robstride on two CAN buses + base IMU |

Every variant comes from the same Onshape document with a different `Configuration=`.

## Hardware backends

A variant with a `ros2_control` block selects its backend through two xacro args, the
convention the [Universal Robots description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/ros2/urdf/ur.ros2_control.xacro)
uses: one boolean per non-real backend, with real hardware as the fallback.

| `sim_mujoco` | `use_mock_hardware` | Backend |
|---|---|---|
| `false` | `false` | `humanoid_devices_robstride/RobstrideSystem`, one block per CAN bus |
| `false` | `true` | `mock_components/GenericSystem` |
| `true` | any | `mujoco_ros2_control/MujocoSystem` |

`sim_mujoco` wins over `use_mock_hardware`. Note that `sim_mujoco` is unrelated to the
`use_sim_time` node parameter, which controls the clock.

The switches choose a plugin and nothing else. The joint macros are backend-agnostic and
always emit their hardware params, because every backend ignores the params it does not
know. Real hardware needs one `<ros2_control>` block per physical CAN bus, so the real
path emits several blocks while the mock and MuJoCo paths share one combined block.

## CAD source

Every description is generated from an Onshape assembly (document
`e9ee61a2e2678af2088d9f31`) by the `robot_assets` tool. See
[Re-generating from CAD](#re-generating-from-cad). The files under `robots/<variant>/` are
build artifacts. Do not hand-edit them. Change the `cad/` inputs and regenerate.

## Usage

### Simulation and RL (Python, no ROS toolchain)

```bash
uv add git+https://github.com/Berkeley-Humanoids/Lite-Description.git
```

```python
from robot_assets import load

urdf_path = load("robots/lite/urdf/lite.urdf")               # Isaac Lab
mjcf_path = load("robots/lite_dummy/mjcf/lite_dummy.xml")    # MuJoCo
```

`load()` fetches the requested variant's subtree from this GitHub repo and caches it. No
ROS install is required.

### ROS 2

`lite_description` is a standard `ament_cmake` package whose `package.xml` sits at the
repo root. Build it in a ROS 2 workspace, or pull it with `vcs` or
`humanoid_control.repos` from `Humanoid Control`, then run `colcon build`. Downstream,
`robot_state_publisher` runs xacro on
`robots/<variant>/xacro/<variant>.urdf.xacro`, and
`package://lite_description/robots/<variant>/meshes/visual/...` resolves after install.

## Repository layout

```
Lite-Description/                  # repo root == ament package "lite_description"
  package.xml  CMakeLists.txt      # ament (colcon); installs robots/<variant>/...
  pyproject.toml                   # pip/uv: builds the robot_assets Python module
  robot_assets/                    # Python module: load() and the CAD->assets generator
    actuators/                     #   actuator spec tables (velocity/effort/armature)
    workflow/                      #   the generator stages
  robots/                          # per-variant assets (franka_description-style subdir)
    <variant>/
      xacro/                       #   ROS entry (GENERATED)
        <variant>.urdf.xacro       #     assembly: args, includes, instantiation
        <variant>.description.xacro  #   model macro: kinematics, ${mesh_root}, base_link
        <variant>.ros2_control.xacro #   hardware macros: joints, groups, backends
      urdf/<variant>.urdf          #   flat URDF (GENERATED; the kinematic HUB)
      mjcf/<variant>.xml           #   MJCF (GENERATED; MuJoCo training + deployment sim)
      meshes/visual/*.stl          #   one shared mesh copy
      cad/                         #   generation INPUTS (not installed):
        config.json                #     Onshape document + export options
        joint_properties.json      #     sim tuning: armature / friction / effort_limit
        physics.json               #     MJCF <option>, freejoint, IMU, contact (optional)
        ros2_control.json          #     ROS hardware map (optional)
        scad/                      #     collider sources
```

The committed `urdf/<variant>.urdf` is the single kinematic hub. The `mjcf` and `xacro`
stages both derive from it, so the three formats cannot drift apart. The hub is
`base_link`-free, because `base_link` is a ROS and KDL concern that belongs only in the
description xacro. That keeps the finalize stage idempotent and the MJCF rooted at the CAD
root link.

## Re-generating from CAD

```bash
uv sync --extra cad
sudo apt install openscad        # for collider editing (onshape-to-robot)
```

One command produces all three formats from a variant's `cad/` inputs:

```bash
# Full pipeline. The Onshape stage is skipped whenever the URDF hub is committed.
uv run robot-assets-generate lite_dummy

# Re-emit only some stages, after editing physics.json or ros2_control.json:
uv run robot-assets-generate lite_dummy --only mjcf,xacro

# Re-run the Onshape export even though the hub is committed:
uv run robot-assets-generate lite_dummy --force
```

| Stage | Reads | Writes |
|---|---|---|
| `onshape` | `cad/config.json`, `cad/scad/` | `urdf/<variant>.urdf`, `meshes/visual/` |
| `urdf` | `urdf/<variant>.urdf`, `joint_properties.json` | finalized `urdf/<variant>.urdf` (welded, effort harmonised, idempotent) |
| `mjcf` | `urdf/<variant>.urdf`, `joint_properties.json`, `physics.json` | `mjcf/<variant>.xml` |
| `xacro` | `urdf/<variant>.urdf`, `ros2_control.json` | `xacro/<variant>.*.xacro` (`base_link` injected here) |
| `package` | — | registers the variant in the repo-root `CMakeLists.txt` |

The Onshape stage costs about 1000 API requests, which is why it is skipped by default.

A variant without a `ros2_control.json` generates a model-only package: a description
macro and a thin assembly, with no `<ros2_control>` block.

The MJCF carries actuators but no `<sensor>` block unless `physics.json` asks for the base
IMU. Joint state is read from the sim data, and `mujoco_ros2_control` aborts on a sensor
that is not backed by a `<site>`.

### Editing colliders (OpenSCAD)

```bash
uv run robot-assets-onshape-to-urdf lite_dummy --keep-assets
cd robots/lite_dummy/cad/assets/
uv run onshape-to-robot-edit-shape ./chest.stl
```

## Tests

```bash
uv run pytest
```

The suite covers left/right symmetry, URDF-to-MJCF parity, inertial plausibility, mesh
existence, xacro well-formedness and generator determinism. The `xacro` and `check_urdf`
expansion checks run only where those tools are installed, so they are skipped in the
plain `uv` environment and are exercised by the ROS CI job instead.

# Robot Descriptions

URDF and MuJoCo (MJCF) descriptions for humanoid robots and other assets, plus CLI tools to regenerate them from Onshape using [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) and to convert URDF to MJCF.

CLI implementations live under `robot_descriptions/workflow/`.

## Contents

Collision geometry in the shipped URDF uses primitives and/or merged meshes from the export pipeline; **visual meshes** are loaded from `robots/<robot>/meshes/` (paths relative to the URDF).

## Quick Start

1. setting up dependency.

    ```bash
    sudo apt install openscad
    uv sync
    ```

2. Example usage:

    ```python
    from robot_descriptions import load_asset

    urdf_path = load_asset("robots/miku/urdf/miku.urdf")
    ```

3. Regenerate files from Onshape.

    ```bash
    uv run robot-descriptions-export-onshape-to-urdf ./robots/<robot>/urdf/config.json
    ```

    Use `--keep-temp-files` if you need to inspect intermediate `assets/` output.

4. Edit collision shapes (OpenSCAD)

    To edit the collider shapes, use `--keep-temp-files` for the onshape export script. Then, use the following commands to edit:

    ```bash
    cd ./robots/<robot>/urdf/assets/
    uv run onshape-to-robot-edit-shape ./chest.stl
    ```

5. URDF -> MJCF

    Joint actuator and MJCF joint tuning are loaded from `robots/<robot>/urdf/joint_properties.json`.

    ```bash
    uv run robot-descriptions-convert-urdf-to-mjcf ./robots/<robot>/urdf/<robot_name>.urdf ./robots/<robot>/mjcf/<robot_name>.xml
    ```

    For floating base robots:

    ```bash
    uv run robot-descriptions-convert-urdf-to-mjcf ./robots/<robot>/urdf/<robot_name>.urdf ./robots/<robot>/mjcf/<robot_name>.xml --freejoint
    ```

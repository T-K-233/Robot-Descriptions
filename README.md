# Robot Descriptions

URDF and MuJoCo (MJCF) descriptions for humanoid robots and other assets, plus scripts to regenerate them from Onshape using [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) and to convert URDF to MJCF.

## Contents

Collision geometry in the shipped URDF uses primitives and/or merged meshes from the export pipeline; **visual meshes** are loaded from `robots/<robot>/meshes/` (paths relative to the URDF).

## Quick Start

1. setting up dependency.

    ```bash
    sudo apt install openscad
    uv sync
    ```

2. Regenerate files from Onshape.

    ```bash
    uv run ./scripts/export_onshape_to_urdf.py ./robots/<robot>/urdf/config.json
    ```

    Use `--keep-temp-files` if you need to inspect intermediate `assets/` output.

3. Edit collision shapes (OpenSCAD)

    To edit the collider shapes, use `--keep-temp-files` for the onshape export script. Then, use the following commands to edit:

    ```bash
    cd ./robots/<robot>/urdf/assets/
    uv run onshape-to-robot-edit-shape ./chest.stl
    ```

4. URDF → MJCF

    Joint actuator and MJCF joint tuning are loaded from `robots/<robot>/urdf/joint_properties.json`.

    ```bash
    uv run ./scripts/convert_urdf_to_mjcf.py ./robots/<robot>/urdf/<robot_name>.urdf ./robots/<robot>/mjcf/<robot_name>.xml
    ```

    For floating base robots:

    ```bash
    uv run ./scripts/convert_urdf_to_mjcf.py ./robots/<robot>/urdf/<robot_name>.urdf ./robots/<robot>/mjcf/<robot_name>.xml --freejoint
    ```

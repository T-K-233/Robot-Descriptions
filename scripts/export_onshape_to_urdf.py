import argparse
import json
import os
from pathlib import Path
import shutil


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Script to generate URDF file from onshape CAD project.",
    )
    parser.add_argument("config", type=str, help="Path to the config file.")
    parser.add_argument(
        "--keep-temp-files",
        action="store_true",
        help="Keep the temporary files.",
        default=False,
    )
    args = parser.parse_args()

    # check if the config file exists
    config_file_path = Path(args.config)
    if not config_file_path.exists():
        raise FileNotFoundError(f"Config file {config_file_path} does not exist!")

    robot_name = json.load(open(config_file_path))["output_filename"]
    urdf_dir = config_file_path.parent
    robot_dir = urdf_dir.parent
    scad_dir = robot_dir / "scad"

    # copy all the predefined scad files to the urdf assets directory
    # for onshape-to-robot to generate custom colliders
    if scad_dir.exists():
        # Create assets directory if it doesn't exist
        assets_dir = urdf_dir / "assets"
        assets_dir.mkdir(exist_ok=True)

        for file in scad_dir.iterdir():
            shutil.copy(file, assets_dir / file.name)

    # invoke onshape-to-robot to generate the urdf file
    os.system(f"onshape-to-robot {urdf_dir}")

    # copy everything under merged/ directory to the assets directory
    if (urdf_dir / "assets" / "merged").exists():
        shutil.copytree(
            urdf_dir / "assets" / "merged",
            robot_dir / "meshes",
            dirs_exist_ok=True,
        )

    # delete the assets directory
    if not args.keep_temp_files:
        shutil.rmtree(urdf_dir / "assets", ignore_errors=True)

    # modify the urdf to use the mesh from the parent meshes directory
    with open(urdf_dir / f"{robot_name}.urdf", "r") as file:
        content = file.read()

    content = content.replace("assets/merged/", "../meshes/")
    content = content.replace("package://", "")

    with open(urdf_dir / f"{robot_name}.urdf", "w") as file:
        file.write(content)

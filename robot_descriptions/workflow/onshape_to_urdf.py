"""
uv run robot-descriptions-export-onshape-to-urdf ./robots/<robot>/urdf/config.json
"""

import argparse
import json
from pathlib import Path
import shutil
import subprocess


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
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
    parser.add_argument("--convert", action="store_true", help="Convert from local robot.pkl")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    # check if the config file exists
    config_file_path = Path(args.config)
    if not config_file_path.exists():
        raise FileNotFoundError(f"Config file {config_file_path} does not exist!")

    robot_name = json.loads(config_file_path.read_text())["output_filename"]
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
    if args.convert:
        subprocess.run(["onshape-to-robot", str(urdf_dir), "--convert"], check=True)
    else:
        subprocess.run(["onshape-to-robot", str(urdf_dir)], check=True)

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
    urdf_path = urdf_dir / f"{robot_name}.urdf"
    content = urdf_path.read_text()

    content = content.replace("assets/merged/", "../meshes/")
    content = content.replace("package://", "")

    urdf_path.write_text(content)


if __name__ == "__main__":
    main()

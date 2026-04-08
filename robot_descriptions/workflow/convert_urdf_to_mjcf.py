"""
uv run robot-descriptions-convert-urdf-to-mjcf ./robots/<robot>/urdf/<robot_name>.urdf ./robots/<robot>/mjcf/<robot_name>.xml [--freejoint]
"""

import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET


MUJOCO_COMPILER_OPTIONS = {
    "meshdir": "../meshes/",
    "discardvisual": "false",
    "fusestatic": "false",
    "angle": "radian",
}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert a URDF file to MJCF.")
    parser.add_argument("input", type=str, help="Path to the URDF file.")
    parser.add_argument("output", type=str, help="Path to the MJCF XML file.")
    parser.add_argument(
        "--freejoint",
        action="store_true",
        help="Add a free joint under the first body element.",
    )
    return parser.parse_args(argv)


def extract_mesh_folders(urdf_file_path: Path) -> list[Path]:
    mesh_folders: list[Path] = []
    root = ET.parse(urdf_file_path).getroot()

    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue

        mesh_folder = Path(filename).parent
        if str(mesh_folder) == "." or mesh_folder in mesh_folders:
            continue

        mesh_folders.append(mesh_folder)

    return mesh_folders


def create_temp_directories(urdf_path: Path) -> tuple[Path, Path, Path]:
    temp_dir = urdf_path.parent.parent / "temp"
    if temp_dir.exists():
        shutil.rmtree(temp_dir)

    temp_urdf_dir = temp_dir / "urdf"
    temp_meshes_dir = temp_dir / "meshes"
    temp_urdf_dir.mkdir(parents=True, exist_ok=True)
    temp_meshes_dir.mkdir(parents=True, exist_ok=True)
    return temp_dir, temp_urdf_dir, temp_meshes_dir


def add_mujoco_compiler_tag(urdf_file_path: Path) -> None:
    tree = ET.parse(urdf_file_path)
    root = tree.getroot()

    mujoco_tag = root.find("mujoco")
    if mujoco_tag is None:
        mujoco_tag = ET.Element("mujoco")
        root.append(mujoco_tag)

    compiler_tag = mujoco_tag.find("compiler")
    if compiler_tag is None:
        compiler_tag = ET.Element("compiler")
        mujoco_tag.append(compiler_tag)

    for key, value in MUJOCO_COMPILER_OPTIONS.items():
        compiler_tag.set(key, value)

    tree.write(urdf_file_path, encoding="utf-8", xml_declaration=True)


def copy_meshes_to_temp(
    urdf_path: Path,
    mesh_folders: list[Path],
    temp_meshes_dir: Path,
    temp_urdf_path: Path,
) -> list[tuple[str, str]]:
    content = temp_urdf_path.read_text()
    changed_mesh_paths: list[tuple[str, str]] = []

    for pattern in ("*.stl", "*.STL"):
        for meshes_path in mesh_folders:
            full_meshes_path = urdf_path.parent / meshes_path
            for file_path in full_meshes_path.rglob(pattern):
                if not file_path.is_file():
                    continue

                target_filename = file_path.name
                if meshes_path.name != "meshes":
                    original_reference = f"{meshes_path}/{file_path.name}"
                    rewritten_reference = (
                        f"{meshes_path.parent}/{meshes_path.name}_{file_path.name}"
                    )
                    target_filename = f"{meshes_path.name}_{target_filename}"
                    changed_mesh_paths.append((original_reference, target_filename))
                    content = content.replace(original_reference, rewritten_reference)

                print(f"Found mesh file: {file_path} -> {target_filename}")
                shutil.copy2(file_path, temp_meshes_dir / target_filename)

    temp_urdf_path.write_text(content)
    return changed_mesh_paths


def launch_mujoco_viewer(temp_urdf_dir: Path, urdf_filename: str) -> Path:
    print("Launching mujoco viewer...")
    print("Please click the 'Save XML' button in the viewer to save the MJCF file and then close the viewer.")

    subprocess.run(
        [sys.executable, "-m", "mujoco.viewer", "--mjcf", urdf_filename],
        cwd=temp_urdf_dir,
        check=True,
    )

    temp_xml_path = temp_urdf_dir / "mjmodel.xml"
    if not temp_xml_path.exists():
        raise FileNotFoundError(
            f"Expected MuJoCo to save MJCF to {temp_xml_path}, but the file was not created.",
        )

    return temp_xml_path


def load_joint_properties(urdf_path: Path) -> dict:
    joint_properties_path = urdf_path.parent / "joint_properties.json"
    if not joint_properties_path.exists():
        raise FileNotFoundError(
            f"Joint properties file {joint_properties_path} does not exist!",
        )

    return json.loads(joint_properties_path.read_text())


def resolve_joint_properties(joint_name: str, joint_properties: dict) -> dict:
    if joint_name in joint_properties:
        return joint_properties[joint_name]

    for pattern, config in joint_properties.items():
        try:
            if re.match(pattern, joint_name):
                return config
        except re.error:
            print(f"Pattern {pattern} is not a valid regex, skipping")

    raise ValueError(
        f"No joint properties found for joint '{joint_name}'. "
        "Please add an exact or regex entry to joint_properties.json.",
    )


def require_joint_attribute(joint_name: str, joint_config: dict, attribute_name: str):
    if attribute_name not in joint_config:
        raise ValueError(
            f"Joint '{joint_name}' is missing required attribute '{attribute_name}' "
            "in joint_properties.json.",
        )

    return joint_config[attribute_name]


def ensure_section(root: ET.Element, tag_name: str, before_tag_name: str | None = None) -> ET.Element:
    section = root.find(tag_name)
    if section is not None:
        section.clear()
        return section

    section = ET.Element(tag_name)
    if before_tag_name is None:
        root.append(section)
        return section

    before_section = root.find(before_tag_name)
    if before_section is not None:
        root.insert(list(root).index(before_section), section)
    else:
        root.append(section)

    return section


def format_motor_forcerange_from_effort_limit(effort_limit) -> str:
    """Build MuJoCo motor `forcerange` as symmetric ±|effort_limit| (e.g. Nm for revolute joints)."""
    if isinstance(effort_limit, (int, float)):
        mag = abs(effort_limit)
        return f"-{mag} {mag}"

    raise ValueError(
        f"effort_limit must be a number, got {type(effort_limit).__name__}: {effort_limit!r}",
    )


def add_actuators_and_sensors(xml_file_path: Path, joint_properties: dict) -> None:
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    joints: list[str] = []
    for joint in root.iter("joint"):
        joint_name = joint.get("name")
        if not joint_name:
            continue

        actuatorfrcrange = joint.get("actuatorfrcrange")
        if actuatorfrcrange:
            joints.append(joint_name)

    if not joints:
        print("No joints with actuatorfrcrange found in XML")
        return

    actuator_section = ensure_section(root, "actuator", before_tag_name="sensor")
    sensor_section = ensure_section(root, "sensor", before_tag_name="equality")

    for joint_name in joints:
        motor = ET.Element("motor")
        motor.set("name", joint_name)
        motor.set("joint", joint_name)

        joint_config = resolve_joint_properties(joint_name, joint_properties)
        effort_limit = require_joint_attribute(joint_name, joint_config, "effort_limit")
        motor.set(
            "forcerange",
            format_motor_forcerange_from_effort_limit(effort_limit),
        )

        actuator_section.append(motor)

    for joint_name in joints:
        jointpos = ET.Element("jointpos")
        jointpos.set("name", f"{joint_name}_pos")
        jointpos.set("joint", joint_name)
        sensor_section.append(jointpos)

    for joint_name in joints:
        jointvel = ET.Element("jointvel")
        jointvel.set("name", f"{joint_name}_vel")
        jointvel.set("joint", joint_name)
        sensor_section.append(jointvel)

    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)


def add_freejoint(xml_file_path: Path) -> None:
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    worldbody = root.find("worldbody")
    if worldbody is None:
        print("No worldbody found in XML")
        return

    first_body = worldbody.find("body")
    if first_body is None:
        print("No body element found in worldbody")
        return

    freejoint = ET.Element("joint")
    freejoint.set("name", "floating_base_joint")
    freejoint.set("type", "free")
    freejoint.set("limited", "false")
    freejoint.set("actuatorfrclimited", "false")
    first_body.insert(0, freejoint)

    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)
    print("Added floating_base_joint to first body element")


def apply_joint_properties(xml_file_path: Path, joint_properties: dict) -> None:
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    for joint in root.iter("joint"):
        joint_name = joint.get("name")
        if not joint_name or joint.get("type") == "free":
            continue

        joint_config = resolve_joint_properties(joint_name, joint_properties)
        friction_loss = require_joint_attribute(joint_name, joint_config, "friction_loss")
        armature = require_joint_attribute(joint_name, joint_config, "armature")
        joint.set("frictionloss", str(friction_loss))
        joint.set("armature", str(armature))

    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)


def write_output_xml(
    temp_xml_path: Path,
    output_xml_path: Path,
    changed_mesh_paths: list[tuple[str, str]],
) -> None:
    content = temp_xml_path.read_text()

    for original_reference, rewritten_reference in changed_mesh_paths:
        content = content.replace(rewritten_reference, original_reference)

    output_xml_path.parent.mkdir(parents=True, exist_ok=True)
    output_xml_path.write_text(content)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    urdf_path = Path(args.input)
    xml_path = Path(args.output)

    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF file {urdf_path} does not exist!")

    joint_properties = load_joint_properties(urdf_path)
    mesh_folders = extract_mesh_folders(urdf_path)
    temp_dir, temp_urdf_dir, temp_meshes_dir = create_temp_directories(urdf_path)

    try:
        temp_urdf_path = temp_urdf_dir / urdf_path.name
        shutil.copy(urdf_path, temp_urdf_path)
        add_mujoco_compiler_tag(temp_urdf_path)

        changed_mesh_paths = copy_meshes_to_temp(
            urdf_path=urdf_path,
            mesh_folders=mesh_folders,
            temp_meshes_dir=temp_meshes_dir,
            temp_urdf_path=temp_urdf_path,
        )

        temp_xml_path = launch_mujoco_viewer(temp_urdf_dir, urdf_path.name)

        if args.freejoint:
            add_freejoint(temp_xml_path)

        add_actuators_and_sensors(temp_xml_path, joint_properties)
        apply_joint_properties(temp_xml_path, joint_properties)
        write_output_xml(temp_xml_path, xml_path, changed_mesh_paths)
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)

    print(f"MJCF file saved to {xml_path}")
    print("Please open the file and format it with a XML formatter to make it more readable.")


if __name__ == "__main__":
    main()

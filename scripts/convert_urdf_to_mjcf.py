"""
uv run ./scripts/convert_urdf_to_mjcf.py ./robots/miku/urdf/miku.urdf ./robots/miku/mjcf/miku.xml [--freejoint]
"""

import argparse
import os
import re
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET


parser = argparse.ArgumentParser()
parser.add_argument("input", type=str, help="Path to the URDF file.")
parser.add_argument("output", type=str, help="Path to the XML file.")
parser.add_argument("--freejoint", action="store_true", help="Add a free joint under the first body element.")
args = parser.parse_args()

urdf_path = Path(args.input)
xml_path = Path(args.output)


def extract_mesh_folders(urdf_file_path):
    """
    Extract all mesh folder paths from a URDF file.
    """
    mesh_folders = []

    try:
        tree = ET.parse(urdf_file_path)
        root = tree.getroot()

        # Find all mesh elements in the URDF
        for mesh in root.iter("mesh"):
            filename = mesh.get("filename")
            if filename:
                # Extract the directory part of the path
                # e.g., "../meshes/left_shoulder_pitch_visual.stl" -> "../meshes/"
                path_obj = Path(filename)
                parent = path_obj.parent

                # Only add if there's a directory component (not just a filename)
                if str(parent) != ".":
                    folder_path = parent
                    # Add to list if not already present
                    if folder_path not in mesh_folders:
                        mesh_folders.append(folder_path)

    except ET.ParseError as e:
        print(f"Error parsing URDF file: {e}")
    except Exception as e:
        print(f"Error reading URDF file: {e}")

    return mesh_folders


# create temporary directory (clean it first if it exists)
temp_dir = urdf_path.parent.parent / "temp"
if temp_dir.exists():
    shutil.rmtree(temp_dir)
temp_urdf_dir = temp_dir / "urdf"
temp_urdf_dir.mkdir(parents=True, exist_ok=True)
temp_meshes_dir = temp_dir / "meshes"
temp_meshes_dir.mkdir(parents=True, exist_ok=True)

# copy urdf to temporary directory
shutil.copy(urdf_path, temp_urdf_dir / urdf_path.name)

mesh_folders = extract_mesh_folders(urdf_path)


mujoco_compiler_options = {
    "meshdir": "../meshes/",
    "discardvisual": "false",
    "fusestatic": "false",
    "angle": "radian",
}

# add mujoco compiler tag to URDF file
tree = ET.parse(temp_urdf_dir / urdf_path.name)
root = tree.getroot()
mujoco_tag = ET.Element("mujoco")
compiler_tag = ET.Element("compiler")
for key, value in mujoco_compiler_options.items():
    compiler_tag.set(key, value)
mujoco_tag.append(compiler_tag)
root.append(mujoco_tag)
tree.write(temp_urdf_dir / urdf_path.name, encoding="utf-8", xml_declaration=True)

# read the content back for further string replacements
with open(temp_urdf_dir / urdf_path.name, "r") as file:
    content = file.read()

changed_mesh_paths = []

# recursively find all .stl and .STL files in meshes directory and copy them to temp folder
for pattern in ["*.stl", "*.STL"]:
    for meshes_path in mesh_folders:
        full_meshes_path = urdf_path.parent / meshes_path
        for file_path in full_meshes_path.rglob(pattern):
            if file_path.is_file():
                # add folder name as prefix for files in left/ and right/ subdirectories
                target_filename = file_path.name

                if meshes_path.name != "meshes":
                    # external mesh files
                    target_filename = f"{meshes_path.name}_{target_filename}"
                    original = f"{meshes_path}/{file_path.name}"
                    new = f"{meshes_path.parent}/{meshes_path.name}_{file_path.name}"
                    changed_mesh_paths.append((original, f"{meshes_path.name}_{file_path.name}"))
                    content = content.replace(original, new)

                # copy .stl/.STL file directly to temp directory with modified filename
                print(f"Found mesh file: {file_path} -> {target_filename}")
                shutil.copy2(file_path, temp_meshes_dir / target_filename)


with open(temp_urdf_dir / urdf_path.name, "w") as file:
    file.write(content)

# create output directory for MJCF file
xml_path.parent.mkdir(parents=True, exist_ok=True)


# print(changed_mesh_paths)

# set currnet working directory to temp directory
cwd = os.getcwd()
os.chdir(temp_urdf_dir)

print("Launching mujoco viewer...")
print("Please click the 'Save XML' button in the viewer to save the MJCF file and then close the viewer.")

# run mujoco viewer with URDF. mujoco will convert the URDF to MJCF automatically
os.system(f"python -m mujoco.viewer --mjcf {urdf_path.name}")

# restore current working directory
os.chdir(cwd)

# mujoco will save file as "mjmodel.xml"
temp_xml_path = temp_urdf_dir / "mjmodel.xml"


def add_actuators_and_sensors(xml_file_path):
    """
    Automatically add actuator motors and sensors to the MJCF XML file
    based on joint names found in the file.
    """
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    # Find all joints in the XML
    joints = []
    for joint in root.iter("joint"):
        joint_name = joint.get("name")
        if joint_name:
            actuatorfrcrange = joint.get("actuatorfrcrange")
            if actuatorfrcrange:
                joints.append((joint_name, actuatorfrcrange))

    if not joints:
        print("No joints with actuatorfrcrange found in XML")
        return

    # Find or create actuator section
    actuator_section = root.find("actuator")
    if actuator_section is None:
        actuator_section = ET.Element("actuator")
        # Insert before sensor section if it exists, otherwise append
        sensor_section = root.find("sensor")
        if sensor_section is not None:
            root.insert(list(root).index(sensor_section), actuator_section)
        else:
            root.append(actuator_section)
    else:
        # Clear existing actuators
        actuator_section.clear()

    # Find or create sensor section
    sensor_section = root.find("sensor")
    if sensor_section is None:
        sensor_section = ET.Element("sensor")
        # Insert before equality section if it exists, otherwise append
        equality_section = root.find("equality")
        if equality_section is not None:
            root.insert(list(root).index(equality_section), sensor_section)
        else:
            root.append(sensor_section)
    else:
        # Clear existing sensors
        sensor_section.clear()

    # Add motors for each joint
    for joint_name, actuatorfrcrange in joints:
        motor = ET.Element("motor")
        motor.set("name", joint_name)
        motor.set("joint", joint_name)
        motor.set("forcerange", "-10 10")  # TODO: correctly set actuator force range
        actuator_section.append(motor)

    # Add jointpos and jointvel sensors for each joint
    for joint_name, _ in joints:
        # Add joint position sensor
        jointpos = ET.Element("jointpos")
        jointpos.set("name", f"{joint_name}_pos")
        jointpos.set("joint", joint_name)
        sensor_section.append(jointpos)

    for joint_name, _ in joints:
        # Add joint velocity sensor
        jointvel = ET.Element("jointvel")
        jointvel.set("name", f"{joint_name}_vel")
        jointvel.set("joint", joint_name)
        sensor_section.append(jointvel)

    # Write the updated XML
    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)


def add_freejoint(xml_file_path):
    """Add a free joint as the first child of the first body element."""
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


# Add freejoint if requested
if args.freejoint:
    add_freejoint(temp_xml_path)

# Add actuators and sensors to the generated XML
add_actuators_and_sensors(temp_xml_path)


def add_joint_friction_loss_and_armature(xml_file_path, config):
    """
    Add frictionloss and armature attributes to joints based on the config dict.

    Args:
        xml_file_path: Path to the XML file to modify
        config: Dictionary mapping joint names to their friction_loss and armature values
                Example: {
                    "joint_name": {
                        "friction_loss": 0.05,
                        "armature": 0.001
                    }
                }
    """
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    # Find all joints in the XML
    for joint in root.iter("joint"):
        joint_name = joint.get("name")
        if not joint_name:
            continue

        joint_config = None

        # First try exact match
        if joint_name in config:
            joint_config = config[joint_name]
        else:
            # Try regex matching
            for pattern, config_value in config.items():
                try:
                    if re.match(pattern, joint_name):
                        joint_config = config_value
                        break
                except re.error:
                    # If pattern is not a valid regex, skip it
                    print(f"Pattern {pattern} is not a valid regex, skipping")
                    continue

        if joint_config:
            # Add frictionloss if specified in config
            if "friction_loss" in joint_config:
                joint.set("frictionloss", str(joint_config["friction_loss"]))

            # Add armature if specified in config
            if "armature" in joint_config:
                joint.set("armature", str(joint_config["armature"]))
        else:
            print(f"Joint {joint_name} not found in config")
            joint.set("frictionloss", "0.05")
            joint.set("armature", "0.001")

    # Write the updated XML
    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)

config = {
    "(left|right)_shoulder_(roll|pitch|yaw)": {
        "friction_loss": 0.1,
        "armature": 0.01,
    },
    "(left|right)_elbow_pitch": {
        "friction_loss": 0.1,
        "armature": 0.01,
    },
    "(left|right)_wrist_(yaw|roll|pitch)": {
        "friction_loss": 0.1,
        "armature": 0.01,
    },
    "(left|right)_finger(1|2|3|4|5)_joint(1|2|3|4)": {
        "friction_loss": 0.1,
        "armature": 0.01,
    },
    "(left|right)_hip_(roll|pitch|yaw)": {
        "friction_loss": 0.1,
        "armature": 0.01,
    },
    "(left|right)_knee_pitch": {
        "friction_loss": 0.1,
        "armature": 0.01,
    },
    "(left|right)_ankle_(yaw|roll|pitch)": {
        "friction_loss": 0.1,
        "armature": 0.01,
    },
    "waist_(roll|pitch|yaw)": {
        "friction_loss": 0.1,
        "armature": 0.01,
    },
    "neck_(roll|pitch|yaw)": {
        "friction_loss": 0.1,
        "armature": 0.01,
    },
}

add_joint_friction_loss_and_armature(temp_xml_path, config)


with open(temp_xml_path, "r") as file:
    content = file.read()

# restore the original folder structure for mesh files
for original, new in changed_mesh_paths:
    content = content.replace(new, original)

with open(xml_path, "w") as file:
    file.write(content)

print(f"MJCF file saved to {xml_path}")
print("Please open the file and format it with a XML formatter to make it more readable.")

# delete temporary directory
shutil.rmtree(temp_dir)

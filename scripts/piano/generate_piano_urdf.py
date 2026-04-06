"""
uv run ./scripts/piano/generate_piano_urdf.py
"""

from pathlib import Path

import consts


def build(add_actuators: bool = False) -> str:
    """Programatically build a piano URDF.

    Args:
        add_actuators: Whether to add actuators to the piano keys.

    Returns:
        URDF XML string.
    """
    urdf_lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        '<robot name="piano">',
        '  <material name="white">',
        f'    <color rgba="{consts.WHITE_KEY_COLOR[0]} {consts.WHITE_KEY_COLOR[1]} {consts.WHITE_KEY_COLOR[2]} {consts.WHITE_KEY_COLOR[3]}" />',
        '  </material>',
        '  <material name="black">',
        f'    <color rgba="{consts.BLACK_KEY_COLOR[0]} {consts.BLACK_KEY_COLOR[1]} {consts.BLACK_KEY_COLOR[2]} {consts.BLACK_KEY_COLOR[3]}" />',
        '  </material>',
        '',
        '  <!-- Base link -->',
        '  <link name="base">',
        '  </link>',
        '',
        '  <!-- Piano body -->',
        '  <link name="body">',
        '    <visual>',
        f'      <origin xyz="{consts.BASE_1_POS[0]} {consts.BASE_1_POS[1]} {consts.BASE_1_POS[2]}" rpy="0 0 0" />',
        '      <geometry>',
        f'        <box size="{consts.BASE_1_SIZE[0]*2} {consts.BASE_1_SIZE[1]*2} {consts.BASE_1_SIZE[2]*2}" />',
        '      </geometry>',
        '    </visual>',
        '    <visual>',
        f'      <origin xyz="{consts.BASE_2_POS[0]} {consts.BASE_2_POS[1]} {consts.BASE_2_POS[2]}" rpy="0 0 0" />',
        '      <geometry>',
        f'         <box size="{consts.BASE_2_SIZE[0]*2} {consts.BASE_2_SIZE[1]*2} {consts.BASE_2_SIZE[2]*2}" />',
        '      </geometry>',
        '    </visual>',
        '    <collision>',
        f'      <origin xyz="{consts.BASE_1_POS[0]} {consts.BASE_1_POS[1]} {consts.BASE_1_POS[2]}" rpy="0 0 0" />',
        '      <geometry>',
        f'         <box size="{consts.BASE_1_SIZE[0]*2} {consts.BASE_1_SIZE[1]*2} {consts.BASE_1_SIZE[2]*2}" />',
        '      </geometry>',
        '    </collision>',
        '    <collision>',
        f'      <origin xyz="{consts.BASE_2_POS[0]} {consts.BASE_2_POS[1]} {consts.BASE_2_POS[2]}" rpy="0 0 0" />',
        '      <geometry>',
        f'         <box size="{consts.BASE_2_SIZE[0]*2} {consts.BASE_2_SIZE[1]*2} {consts.BASE_2_SIZE[2]*2}" />',
        '      </geometry>',
        '    </collision>',
        '    <inertial>',
        f'      <origin xyz="{consts.BASE_1_POS[0]} {consts.BASE_1_POS[1]} {consts.BASE_1_POS[2]}" rpy="0 0 0" />',
        '      <mass value="10.0" />',
        '      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />',
        '    </inertial>',
        '  </link>',
        '',
        '  <joint name="base_body_joint" type="fixed">',
        '    <parent link="base" />',
        '    <child link="body" />',
        '    <origin xyz="0 0 0" rpy="0 0 0" />',
        '  </joint>',
        ''
    ]

    # Generate white key links and joints
    for i in range(consts.NUM_WHITE_KEYS):
        y_coord = (
            -consts.PIANO_KEY_TOTAL_WIDTH * 0.5
            + consts.WHITE_KEY_WIDTH * 0.5
            + i * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
        )

        key_index = consts.WHITE_KEY_INDICES[i]
        key_name = f"key_{key_index}"

        # Calculate joint origin (at the back of the key)
        joint_origin_x = consts.WHITE_KEY_X_OFFSET + consts.WHITE_KEY_LENGTH / 2 - consts.WHITE_KEY_TOTAL_LENGTH
        joint_origin_y = y_coord
        joint_origin_z = consts.WHITE_KEY_Z_OFFSET

        # Calculate visual/collision origin (center of key)
        visual_origin_x = consts.WHITE_KEY_TOTAL_LENGTH - consts.WHITE_KEY_LENGTH + consts.WHITE_KEY_LENGTH / 2
        visual_origin_y = 0
        visual_origin_z = 0

        # Calculate inertia values for a box
        mass = consts.WHITE_KEY_MASS
        lx, ly, lz = consts.WHITE_KEY_LENGTH, consts.WHITE_KEY_WIDTH, consts.WHITE_KEY_HEIGHT
        ixx = mass * (ly**2 + lz**2) / 12
        iyy = mass * (lx**2 + lz**2) / 12
        izz = mass * (lx**2 + ly**2) / 12

        # Add link
        urdf_lines.extend([
            f'  <link name="{key_name}">',
            '    <visual>',
            f'      <origin xyz="{visual_origin_x} {visual_origin_y} {visual_origin_z}" rpy="0 0 0" />',
            '      <geometry>',
            f'        <box size="{lx} {ly} {lz}" />',
            '      </geometry>',
            '      <material name="white" />',
            '    </visual>',
            '    <collision>',
            f'      <origin xyz="{visual_origin_x} {visual_origin_y} {visual_origin_z}" rpy="0 0 0" />',
            '      <geometry>',
            f'        <box size="{lx} {ly} {lz}" />',
            '      </geometry>',
            '    </collision>',
            '    <inertial>',
            f'      <origin xyz="{visual_origin_x} {visual_origin_y} {visual_origin_z}" rpy="0 0 0" />',
            f'      <mass value="{mass}" />',
            f'      <inertia ixx="{ixx}" ixy="0.0" ixz="0.0" iyy="{iyy}" iyz="0.0" izz="{izz}" />',
            '    </inertial>',
            '  </link>',
            '',
            f'  <joint name="{key_name}_joint" type="revolute">',
            '    <parent link="base" />',
            f'    <child link="{key_name}" />',
            f'    <origin xyz="{joint_origin_x} {joint_origin_y} {joint_origin_z}" rpy="0 0 0" />',
            '    <axis xyz="0 1 0" />',
            f'    <limit lower="0" upper="{consts.WHITE_KEY_JOINT_MAX_ANGLE}" effort="10.00" velocity="6.67" />',
            '  </joint>',
            ''
        ])

    # Place the lone black key on the far left.
    y_coord = consts.WHITE_KEY_WIDTH + 0.5 * (
        -consts.PIANO_KEY_TOTAL_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS
    )

    key_index = consts.BLACK_TRIPLET_KEY_INDICES[0]
    key_name = f"key_{key_index}"

    # Calculate joint origin (at the back of the key)
    joint_origin_x = consts.BLACK_KEY_X_OFFSET + consts.BLACK_KEY_LENGTH / 2 - consts.BLACK_KEY_TOTAL_LENGTH
    joint_origin_y = y_coord
    joint_origin_z = consts.BLACK_KEY_Z_OFFSET

    # Calculate visual/collision origin (center of key)
    visual_origin_x = consts.BLACK_KEY_TOTAL_LENGTH - consts.BLACK_KEY_LENGTH + consts.BLACK_KEY_LENGTH / 2
    visual_origin_y = 0
    visual_origin_z = 0

    # Calculate inertia values for a box
    mass = consts.BLACK_KEY_MASS
    lx, ly, lz = consts.BLACK_KEY_LENGTH, consts.BLACK_KEY_WIDTH, consts.BLACK_KEY_HEIGHT
    ixx = mass * (ly**2 + lz**2) / 12
    iyy = mass * (lx**2 + lz**2) / 12
    izz = mass * (lx**2 + ly**2) / 12

    # Add link
    urdf_lines.extend([
        f'  <link name="{key_name}">',
        '    <visual>',
        f'      <origin xyz="{visual_origin_x} {visual_origin_y} {visual_origin_z}" rpy="0 0 0" />',
        '      <geometry>',
        f'        <box size="{lx} {ly} {lz}" />',
        '      </geometry>',
        '      <material name="black" />',
        '    </visual>',
        '    <collision>',
        f'      <origin xyz="{visual_origin_x} {visual_origin_y} {visual_origin_z}" rpy="0 0 0" />',
        '      <geometry>',
        f'        <box size="{lx} {ly} {lz}" />',
        '      </geometry>',
        '    </collision>',
        '    <inertial>',
        f'      <origin xyz="{visual_origin_x} {visual_origin_y} {visual_origin_z}" rpy="0 0 0" />',
        f'      <mass value="{mass}" />',
        f'      <inertia ixx="{ixx}" ixy="0.0" ixz="0.0" iyy="{iyy}" iyz="0.0" izz="{izz}" />',
        '    </inertial>',
        '  </link>',
        '',
        f'  <joint name="{key_name}_joint" type="revolute">',
        '    <parent link="base" />',
        f'    <child link="{key_name}" />',
        f'    <origin xyz="{joint_origin_x} {joint_origin_y} {joint_origin_z}" rpy="0 0 0" />',
        '    <axis xyz="0 1 0" />',
        f'    <limit lower="0" upper="{consts.BLACK_KEY_JOINT_MAX_ANGLE}" effort="10.00" velocity="6.67" />',
        '  </joint>',
        ''
    ])

    # Place the twin black keys.
    n = 0
    for twin_index in consts.TWIN_GROUP_INDICES:
        for j in range(2):
            y_coord = (
                -consts.PIANO_KEY_TOTAL_WIDTH * 0.5
                + (j + 1) * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
                + twin_index
                * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
            )

            key_index = consts.BLACK_TWIN_KEY_INDICES[n]
            key_name = f"key_{key_index}"

            # Calculate joint origin (at the back of the key)
            joint_origin_x = consts.BLACK_KEY_X_OFFSET + consts.BLACK_KEY_LENGTH / 2 - consts.BLACK_KEY_TOTAL_LENGTH
            joint_origin_y = y_coord
            joint_origin_z = consts.BLACK_KEY_Z_OFFSET

            # Calculate visual/collision origin (center of key)
            visual_origin_x = consts.BLACK_KEY_TOTAL_LENGTH - consts.BLACK_KEY_LENGTH + consts.BLACK_KEY_LENGTH / 2
            visual_origin_y = 0
            visual_origin_z = 0

            # Calculate inertia values for a box
            mass = consts.BLACK_KEY_MASS
            lx, ly, lz = consts.BLACK_KEY_LENGTH, consts.BLACK_KEY_WIDTH, consts.BLACK_KEY_HEIGHT
            ixx = mass * (ly**2 + lz**2) / 12
            iyy = mass * (lx**2 + lz**2) / 12
            izz = mass * (lx**2 + ly**2) / 12

            # Add link
            urdf_lines.extend([
                f'  <link name="{key_name}">',
                '    <visual>',
                f'      <origin xyz="{visual_origin_x} {visual_origin_y} {visual_origin_z}" rpy="0 0 0" />',
                '      <geometry>',
                f'        <box size="{lx} {ly} {lz}" />',
                '      </geometry>',
                '      <material name="black" />',
                '    </visual>',
                '    <collision>',
                f'      <origin xyz="{visual_origin_x} {visual_origin_y} {visual_origin_z}" rpy="0 0 0" />',
                '      <geometry>',
                f'        <box size="{lx} {ly} {lz}" />',
                '      </geometry>',
                '    </collision>',
                '    <inertial>',
                f'      <origin xyz="{visual_origin_x} {visual_origin_y} {visual_origin_z}" rpy="0 0 0" />',
                f'      <mass value="{mass}" />',
                f'      <inertia ixx="{ixx}" ixy="0.0" ixz="0.0" iyy="{iyy}" iyz="0.0" izz="{izz}" />',
                '    </inertial>',
                '  </link>',
                '',
                f'  <joint name="{key_name}_joint" type="revolute">',
                '    <parent link="base" />',
                f'    <child link="{key_name}" />',
                f'    <origin xyz="{joint_origin_x} {joint_origin_y} {joint_origin_z}" rpy="0 0 0" />',
                '    <axis xyz="0 1 0" />',
                f'    <limit lower="0" upper="{consts.BLACK_KEY_JOINT_MAX_ANGLE}" effort="10.00" velocity="6.67" />',
                '  </joint>',
                ''
            ])
            n += 1

    # Place the triplet black keys.
    n = 1  # Skip the lone black key.
    for triplet_index in consts.TRIPLET_GROUP_INDICES:
        for j in range(3):
            y_coord = (
                -consts.PIANO_KEY_TOTAL_WIDTH * 0.5
                + (j + 1) * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
                + triplet_index
                * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
            )

            key_index = consts.BLACK_TRIPLET_KEY_INDICES[n]
            key_name = f"key_{key_index}"

            # Calculate joint origin (at the back of the key)
            joint_origin_x = consts.BLACK_KEY_X_OFFSET + consts.BLACK_KEY_LENGTH / 2 - consts.BLACK_KEY_TOTAL_LENGTH
            joint_origin_y = y_coord
            joint_origin_z = consts.BLACK_KEY_Z_OFFSET

            # Calculate visual/collision origin (center of key)
            visual_origin_x = consts.BLACK_KEY_TOTAL_LENGTH - consts.BLACK_KEY_LENGTH + consts.BLACK_KEY_LENGTH / 2
            visual_origin_y = 0
            visual_origin_z = 0

            # Calculate inertia values for a box
            mass = consts.BLACK_KEY_MASS
            lx, ly, lz = consts.BLACK_KEY_LENGTH, consts.BLACK_KEY_WIDTH, consts.BLACK_KEY_HEIGHT
            ixx = mass * (ly**2 + lz**2) / 12
            iyy = mass * (lx**2 + lz**2) / 12
            izz = mass * (lx**2 + ly**2) / 12

            # Add link
            urdf_lines.extend([
                f'  <link name="{key_name}">',
                '    <visual>',
                f'      <origin xyz="{visual_origin_x} {visual_origin_y} {visual_origin_z}" rpy="0 0 0" />',
                '      <geometry>',
                f'        <box size="{lx} {ly} {lz}" />',
                '      </geometry>',
                '      <material name="black" />',
                '    </visual>',
                '    <collision>',
                f'      <origin xyz="{visual_origin_x} {visual_origin_y} {visual_origin_z}" rpy="0 0 0" />',
                '      <geometry>',
                f'        <box size="{lx} {ly} {lz}" />',
                '      </geometry>',
                '    </collision>',
                '    <inertial>',
                f'      <origin xyz="{visual_origin_x} {visual_origin_y} {visual_origin_z}" rpy="0 0 0" />',
                f'      <mass value="{mass}" />',
                f'      <inertia ixx="{ixx}" ixy="0.0" ixz="0.0" iyy="{iyy}" iyz="0.0" izz="{izz}" />',
                '    </inertial>',
                '  </link>',
                '',
                f'  <joint name="{key_name}_joint" type="revolute">',
                '    <parent link="base" />',
                f'    <child link="{key_name}" />',
                f'    <origin xyz="{joint_origin_x} {joint_origin_y} {joint_origin_z}" rpy="0 0 0" />',
                '    <axis xyz="0 1 0" />',
                f'    <limit lower="0" upper="{consts.BLACK_KEY_JOINT_MAX_ANGLE}" effort="10.00" velocity="6.67" />',
                '  </joint>',
                ''
            ])
            n += 1

    # Close the robot tag
    urdf_lines.append('</robot>')

    return '\n'.join(urdf_lines)


if __name__ == "__main__":
    urdf_content = build()

    save_path = Path("./robots/piano/urdf/piano_dep20.urdf")
    # create directory if it doesn't exist
    save_path.parent.mkdir(parents=True, exist_ok=True)

    with open(save_path, "w") as f:
        f.write(urdf_content)

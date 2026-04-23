"""
python -m robot_descriptions.piano.generate_piano_urdf
"""

from pathlib import Path

try:
    from . import consts
except ImportError:
    import consts  # type: ignore[no-redef]


def _key_xml(
    key_name: str,
    joint_name: str,
    color: str,
    mass: float,
    lx: float,
    ly: float,
    lz: float,
    joint_origin: tuple,
    visual_origin: tuple,
    max_angle: float,
    effort: float,
) -> list:
    ixx = mass * (ly ** 2 + lz ** 2) / 12
    iyy = mass * (lx ** 2 + lz ** 2) / 12
    izz = mass * (lx ** 2 + ly ** 2) / 12
    vx, vy, vz = visual_origin
    jx, jy, jz = joint_origin
    return [
        f'  <link name="{key_name}">',
        '    <visual>',
        f'      <origin xyz="{vx} {vy} {vz}" rpy="0 0 0" />',
        '      <geometry>',
        f'        <box size="{lx} {ly} {lz}" />',
        '      </geometry>',
        f'      <material name="{color}" />',
        '    </visual>',
        '    <collision>',
        f'      <origin xyz="{vx} {vy} {vz}" rpy="0 0 0" />',
        '      <geometry>',
        f'        <box size="{lx} {ly} {lz}" />',
        '      </geometry>',
        '    </collision>',
        '    <inertial>',
        f'      <origin xyz="{vx} {vy} {vz}" rpy="0 0 0" />',
        f'      <mass value="{mass}" />',
        f'      <inertia ixx="{ixx}" ixy="0.0" ixz="0.0" iyy="{iyy}" iyz="0.0" izz="{izz}" />',
        '    </inertial>',
        '  </link>',
        '',
        f'  <joint name="{joint_name}" type="revolute">',
        '    <parent link="base" />',
        f'    <child link="{key_name}" />',
        f'    <origin xyz="{jx} {jy} {jz}" rpy="0 0 0" />',
        '    <axis xyz="0 1 0" />',
        f'    <limit lower="0" upper="{max_angle}" effort="{effort}" velocity="6.67" />',
        '  </joint>',
        '',
    ]


def build(variant: str = "dep20") -> str:
    """Programatically build a piano URDF.

    Args:
        variant: The piano variant to build (``"dep20"`` or ``"nikomaku"``).

    Returns:
        URDF XML string.
    """
    match variant:
        case "dep20":
            cfg = consts.PIANO_PARAMS_DEP20
        case "nikomaku":
            cfg = consts.PIANO_PARAMS_NIKOMAKU
        case _:
            raise ValueError(f"Unknown piano variant: {variant}")

    base_1_pos = cfg.BASE_1_POS
    base_1_size = cfg.BASE_1_SIZE
    base_2_pos = cfg.BASE_2_POS
    base_2_size = cfg.BASE_2_SIZE

    urdf_lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        f'<robot name="piano_{variant}">',
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
        f'      <origin xyz="{base_1_pos[0]} {base_1_pos[1]} {base_1_pos[2]}" rpy="0 0 0" />',
        '      <geometry>',
        f'        <box size="{base_1_size[0]*2} {base_1_size[1]*2} {base_1_size[2]*2}" />',
        '      </geometry>',
        '    </visual>',
        '    <visual>',
        f'      <origin xyz="{base_2_pos[0]} {base_2_pos[1]} {base_2_pos[2]}" rpy="0 0 0" />',
        '      <geometry>',
        f'         <box size="{base_2_size[0]*2} {base_2_size[1]*2} {base_2_size[2]*2}" />',
        '      </geometry>',
        '    </visual>',
        '    <collision>',
        f'      <origin xyz="{base_1_pos[0]} {base_1_pos[1]} {base_1_pos[2]}" rpy="0 0 0" />',
        '      <geometry>',
        f'         <box size="{base_1_size[0]*2} {base_1_size[1]*2} {base_1_size[2]*2}" />',
        '      </geometry>',
        '    </collision>',
        '    <collision>',
        f'      <origin xyz="{base_2_pos[0]} {base_2_pos[1]} {base_2_pos[2]}" rpy="0 0 0" />',
        '      <geometry>',
        f'         <box size="{base_2_size[0]*2} {base_2_size[1]*2} {base_2_size[2]*2}" />',
        '      </geometry>',
        '    </collision>',
        '    <inertial>',
        f'      <origin xyz="{base_1_pos[0]} {base_1_pos[1]} {base_1_pos[2]}" rpy="0 0 0" />',
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
        '',
    ]

    # Collect every key with its index so we can emit in sorted order (matches MJCF).
    keys: list[tuple[int, str, float]] = []

    for i in range(consts.NUM_WHITE_KEYS):
        y_coord = (
            -cfg.PIANO_KEY_TOTAL_WIDTH * 0.5
            + cfg.WHITE_KEY_WIDTH * 0.5
            + i * (cfg.WHITE_KEY_WIDTH + cfg.SPACING_BETWEEN_WHITE_KEYS)
        )
        keys.append((consts.WHITE_KEY_INDICES[i], "white", y_coord))

    # Lone black key on the far left.
    y_coord = cfg.WHITE_KEY_WIDTH + 0.5 * (
        -cfg.PIANO_KEY_TOTAL_WIDTH + cfg.SPACING_BETWEEN_WHITE_KEYS
    )
    keys.append((consts.BLACK_TRIPLET_KEY_INDICES[0], "black", y_coord))

    # Twin black keys.
    n = 0
    for twin_index in consts.TWIN_GROUP_INDICES:
        for j in range(2):
            y_coord = (
                -cfg.PIANO_KEY_TOTAL_WIDTH * 0.5
                + (j + 1) * (cfg.WHITE_KEY_WIDTH + cfg.SPACING_BETWEEN_WHITE_KEYS)
                + twin_index * (cfg.WHITE_KEY_WIDTH + cfg.SPACING_BETWEEN_WHITE_KEYS)
            )
            keys.append((consts.BLACK_TWIN_KEY_INDICES[n], "black", y_coord))
            n += 1

    # Triplet black keys.
    n = 1  # Skip the lone black key.
    for triplet_index in consts.TRIPLET_GROUP_INDICES:
        for j in range(3):
            y_coord = (
                -cfg.PIANO_KEY_TOTAL_WIDTH * 0.5
                + (j + 1) * (cfg.WHITE_KEY_WIDTH + cfg.SPACING_BETWEEN_WHITE_KEYS)
                + triplet_index * (cfg.WHITE_KEY_WIDTH + cfg.SPACING_BETWEEN_WHITE_KEYS)
            )
            keys.append((consts.BLACK_TRIPLET_KEY_INDICES[n], "black", y_coord))
            n += 1

    keys.sort(key=lambda k: k[0])

    for index, color, y_coord in keys:
        key_name = f"{color}_key_{index}"
        joint_name = f"{color}_joint_{index}"
        if color == "white":
            joint_origin = (
                cfg.WHITE_KEY_X_OFFSET
                + cfg.WHITE_KEY_LENGTH / 2
                - cfg.WHITE_KEY_TOTAL_LENGTH,
                y_coord,
                cfg.WHITE_KEY_Z_OFFSET,
            )
            visual_origin = (
                cfg.WHITE_KEY_TOTAL_LENGTH
                - cfg.WHITE_KEY_LENGTH
                + cfg.WHITE_KEY_LENGTH / 2,
                0,
                0,
            )
            mass = cfg.WHITE_KEY_MASS
            lx, ly, lz = (
                cfg.WHITE_KEY_LENGTH,
                cfg.WHITE_KEY_WIDTH,
                cfg.WHITE_KEY_HEIGHT,
            )
            max_angle = cfg.WHITE_KEY_JOINT_MAX_ANGLE
        else:
            joint_origin = (
                cfg.BLACK_KEY_X_OFFSET
                + cfg.BLACK_KEY_LENGTH / 2
                - cfg.BLACK_KEY_TOTAL_LENGTH,
                y_coord,
                cfg.BLACK_KEY_Z_OFFSET,
            )
            visual_origin = (
                cfg.BLACK_KEY_TOTAL_LENGTH
                - cfg.BLACK_KEY_LENGTH
                + cfg.BLACK_KEY_LENGTH / 2,
                0,
                0,
            )
            mass = cfg.BLACK_KEY_MASS
            lx, ly, lz = (
                cfg.BLACK_KEY_LENGTH,
                cfg.BLACK_KEY_WIDTH,
                cfg.BLACK_KEY_HEIGHT,
            )
            max_angle = cfg.BLACK_KEY_JOINT_MAX_ANGLE
        urdf_lines.extend(
            _key_xml(
                key_name=key_name,
                joint_name=joint_name,
                color=color,
                mass=mass,
                lx=lx,
                ly=ly,
                lz=lz,
                joint_origin=joint_origin,
                visual_origin=visual_origin,
                max_angle=max_angle,
                effort=cfg.KEY_MAX_TORQUE,
            )
        )

    # Close the robot tag
    urdf_lines.append('</robot>')

    return '\n'.join(urdf_lines)


def main() -> None:
    out_dir = Path("./robots/piano/urdf")
    out_dir.mkdir(parents=True, exist_ok=True)
    variants = [
        ("dep20", consts.PIANO_PARAMS_DEP20),
        ("nikomaku", consts.PIANO_PARAMS_NIKOMAKU),
    ]
    for variant, cfg in variants:
        (out_dir / f"piano_{variant}.urdf").write_text(build(variant))
        print(f"piano_{variant}:")
        print(f"  key-surface to base-bottom height: {cfg.WHITE_KEY_OFFSET_FROM_BASE:.4f} m")
        print(f"  base width:                        {cfg.BASE_WIDTH:.4f} m")
        print(f"  piano key total width:             {cfg.PIANO_KEY_TOTAL_WIDTH:.4f} m")
        print(f"  white key length:                  {cfg.WHITE_KEY_LENGTH:.4f} m")
        print(f"  black key length:                  {cfg.BLACK_KEY_LENGTH:.4f} m")


if __name__ == "__main__":
    main()

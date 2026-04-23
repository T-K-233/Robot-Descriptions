# Copyright 2023 The RoboPianist Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Programatically build a piano MJCF model."""

from pathlib import Path

from dm_control import mjcf
from mujoco_utils import types

try:
    from . import consts
except ImportError:
    import consts  # type: ignore[no-redef]


def build(variant: str = "dep20") -> types.MjcfRootElement:
    """Programatically build a piano MJCF.

    Args:
        variant: The piano variant to build (``"dep20"`` or ``"nikomaku"``).
    """
    match variant:
        case "dep20":
            cfg = consts.PIANO_PARAMS_DEP20
        case "nikomaku":
            cfg = consts.PIANO_PARAMS_NIKOMAKU
        case _:
            raise ValueError(f"Unknown piano variant: {variant}")

    root = mjcf.RootElement()
    root.model = f"piano_{variant}"

    root.compiler.autolimits = True
    root.compiler.angle = "radian"

    # Add materials.
    root.asset.add("material", name="white", rgba=consts.WHITE_KEY_COLOR)
    root.asset.add("material", name="black", rgba=consts.BLACK_KEY_COLOR)

    root.default.geom.type = "box"
    root.default.joint.type = "hinge"
    root.default.joint.axis = [0, 1, 0]
    root.default.site.type = "box"
    root.default.site.group = 4
    root.default.site.rgba = [1, 0, 0, 1]

    # This effectively disables key-key collisions but still allows hand-key collisions,
    # assuming we've kept the default hand contype = conaffinity = 1.
    # See https://mujoco.readthedocs.io/en/latest/computation.html#selection for more
    # details.
    root.default.geom.contype = 0
    root.default.geom.conaffinity = 1

    # White key defaults.
    white_default = root.default.add("default", dclass="white_key")
    white_default.geom.material = "white"
    white_default.geom.size = [
        cfg.WHITE_KEY_LENGTH / 2,
        cfg.WHITE_KEY_WIDTH / 2,
        cfg.WHITE_KEY_HEIGHT / 2,
    ]
    white_default.geom.mass = cfg.WHITE_KEY_MASS
    white_default.site.size = white_default.geom.size
    white_default.joint.pos = [
        cfg.WHITE_KEY_LENGTH / 2 - cfg.WHITE_KEY_TOTAL_LENGTH, 0, 0,
    ]
    white_default.joint.damping = cfg.KEY_DAMPING
    white_default.joint.armature = cfg.KEY_ARMATURE
    white_default.joint.stiffness = cfg.KEY_STIFFNESS
    white_default.joint.springref = cfg.KEY_SPRINGREF
    white_default.joint.range = [0, cfg.WHITE_KEY_JOINT_MAX_ANGLE]

    # Black key defaults.
    black_default = root.default.add("default", dclass="black_key")
    black_default.geom.material = "black"
    black_default.geom.size = [
        cfg.BLACK_KEY_LENGTH / 2,
        cfg.BLACK_KEY_WIDTH / 2,
        cfg.BLACK_KEY_HEIGHT / 2,
    ]
    black_default.site.size = black_default.geom.size
    black_default.geom.mass = cfg.BLACK_KEY_MASS
    black_default.joint.pos = [
        cfg.BLACK_KEY_LENGTH / 2 - cfg.BLACK_KEY_TOTAL_LENGTH, 0, 0,
    ]
    black_default.joint.damping = cfg.KEY_DAMPING
    black_default.joint.armature = cfg.KEY_ARMATURE
    black_default.joint.stiffness = cfg.KEY_STIFFNESS
    black_default.joint.springref = cfg.KEY_SPRINGREF
    black_default.joint.range = [0, cfg.BLACK_KEY_JOINT_MAX_ANGLE]

    # Add parent piano body.
    piano_body = root.worldbody.add("body", name="piano")

    # Add base inside piano body.
    base_body = piano_body.add("body", name="base")
    base_body.add(
        "inertial",
        pos=cfg.BASE_1_POS,
        mass=10.0,
        diaginertia=[0.1, 0.1, 0.1],
    )
    base_body.add(
        "geom",
        name="base_1",
        type="box",
        pos=cfg.BASE_1_POS,
        size=cfg.BASE_1_SIZE,
        rgba=consts.BASE_COLOR,
    )
    base_body.add(
        "geom",
        name="base_2",
        type="box",
        pos=cfg.BASE_2_POS,
        size=cfg.BASE_2_SIZE,
        rgba=consts.BASE_COLOR,
    )

    # These will hold kwargs. We'll subsequently use them to create the actual objects.
    geoms = []
    bodies = []
    joints = []
    sites = []

    for i in range(consts.NUM_WHITE_KEYS):
        y_coord = (
            -cfg.PIANO_KEY_TOTAL_WIDTH * 0.5
            + cfg.WHITE_KEY_WIDTH * 0.5
            + i * (cfg.WHITE_KEY_WIDTH + cfg.SPACING_BETWEEN_WHITE_KEYS)
        )
        bodies.append(
            {
                "name": f"white_key_{consts.WHITE_KEY_INDICES[i]}",
                "pos": [
                    cfg.WHITE_KEY_X_OFFSET,
                    y_coord,
                    cfg.WHITE_KEY_Z_OFFSET,
                ],
            }
        )
        geoms.append(
            {
                "name": f"white_key_geom_{consts.WHITE_KEY_INDICES[i]}",
                "dclass": "white_key",
            }
        )
        joints.append(
            {
                "name": f"white_joint_{consts.WHITE_KEY_INDICES[i]}",
                "dclass": "white_key",
            }
        )
        sites.append(
            {
                "name": f"white_key_site_{consts.WHITE_KEY_INDICES[i]}",
                "dclass": "white_key",
            }
        )

    # Place the lone black key on the far left.
    y_coord = cfg.WHITE_KEY_WIDTH + 0.5 * (
        -cfg.PIANO_KEY_TOTAL_WIDTH + cfg.SPACING_BETWEEN_WHITE_KEYS
    )
    bodies.append(
        {
            "name": f"black_key_{consts.BLACK_TRIPLET_KEY_INDICES[0]}",
            "pos": [
                cfg.BLACK_KEY_X_OFFSET,
                y_coord,
                cfg.BLACK_KEY_Z_OFFSET,
            ],
        }
    )
    geoms.append(
        {
            "name": f"black_key_geom_{consts.BLACK_TRIPLET_KEY_INDICES[0]}",
            "dclass": "black_key",
        }
    )
    joints.append(
        {
            "name": f"black_joint_{consts.BLACK_TRIPLET_KEY_INDICES[0]}",
            "dclass": "black_key",
        }
    )
    sites.append(
        {
            "name": f"black_key_site_{consts.BLACK_TRIPLET_KEY_INDICES[0]}",
            "dclass": "black_key",
        }
    )

    # Place the twin black keys.
    n = 0
    for twin_index in consts.TWIN_GROUP_INDICES:
        for j in range(2):
            y_coord = (
                -cfg.PIANO_KEY_TOTAL_WIDTH * 0.5
                + (j + 1) * (cfg.WHITE_KEY_WIDTH + cfg.SPACING_BETWEEN_WHITE_KEYS)
                + twin_index * (cfg.WHITE_KEY_WIDTH + cfg.SPACING_BETWEEN_WHITE_KEYS)
            )
            bodies.append(
                {
                    "name": f"black_key_{consts.BLACK_TWIN_KEY_INDICES[n]}",
                    "pos": [
                        cfg.BLACK_KEY_X_OFFSET,
                        y_coord,
                        cfg.BLACK_KEY_Z_OFFSET,
                    ],
                }
            )
            geoms.append(
                {
                    "name": f"black_key_geom_{consts.BLACK_TWIN_KEY_INDICES[n]}",
                    "dclass": "black_key",
                }
            )
            joints.append(
                {
                    "name": f"black_joint_{consts.BLACK_TWIN_KEY_INDICES[n]}",
                    "dclass": "black_key",
                }
            )
            sites.append(
                {
                    "name": f"black_key_site_{consts.BLACK_TWIN_KEY_INDICES[n]}",
                    "dclass": "black_key",
                }
            )
            n += 1

    # Place the triplet black keys.
    n = 1  # Skip the lone black key.
    for triplet_index in consts.TRIPLET_GROUP_INDICES:
        for j in range(3):
            y_coord = (
                -cfg.PIANO_KEY_TOTAL_WIDTH * 0.5
                + (j + 1) * (cfg.WHITE_KEY_WIDTH + cfg.SPACING_BETWEEN_WHITE_KEYS)
                + triplet_index * (cfg.WHITE_KEY_WIDTH + cfg.SPACING_BETWEEN_WHITE_KEYS)
            )
            bodies.append(
                {
                    "name": f"black_key_{consts.BLACK_TRIPLET_KEY_INDICES[n]}",
                    "pos": [
                        cfg.BLACK_KEY_X_OFFSET,
                        y_coord,
                        cfg.BLACK_KEY_Z_OFFSET,
                    ],
                }
            )
            geoms.append(
                {
                    "name": f"black_key_geom_{consts.BLACK_TRIPLET_KEY_INDICES[n]}",
                    "dclass": "black_key",
                }
            )
            joints.append(
                {
                    "name": f"black_joint_{consts.BLACK_TRIPLET_KEY_INDICES[n]}",
                    "dclass": "black_key",
                }
            )
            sites.append(
                {
                    "name": f"black_key_site_{consts.BLACK_TRIPLET_KEY_INDICES[n]}",
                    "dclass": "black_key",
                }
            )
            n += 1

    # Sort the elements based on the key number.
    names: list[str] = [body["name"] for body in bodies]  # type: ignore
    indices = sorted(range(len(names)), key=lambda k: int(names[k].split("_")[-1]))
    bodies = [bodies[i] for i in indices]
    geoms = [geoms[i] for i in indices]
    joints = [joints[i] for i in indices]
    sites = [sites[i] for i in indices]

    # Now create the corresponding MJCF elements and add them to the piano body.
    for i in range(len(bodies)):
        body = piano_body.add("body", **bodies[i])
        body.add("geom", **geoms[i])
        body.add("joint", **joints[i])
        body.add("site", **sites[i])

    return root


def main() -> None:
    out_dir = Path("./robots/piano/mjcf")
    out_dir.mkdir(parents=True, exist_ok=True)
    variants = [
        ("dep20", consts.PIANO_PARAMS_DEP20),
        ("nikomaku", consts.PIANO_PARAMS_NIKOMAKU),
    ]
    for variant, cfg in variants:
        root = build(variant)
        (out_dir / f"piano_{variant}.xml").write_text(root.to_xml_string())
        print(f"piano_{variant}:")
        print(f"  key-surface to base-bottom height: {cfg.WHITE_KEY_OFFSET_FROM_BASE:.4f} m")
        print(f"  base width:                        {cfg.BASE_WIDTH:.4f} m")
        print(f"  piano key total width:             {cfg.PIANO_KEY_TOTAL_WIDTH:.4f} m")
        print(f"  white key length:                  {cfg.WHITE_KEY_LENGTH:.4f} m")
        print(f"  black key length:                  {cfg.BLACK_KEY_LENGTH:.4f} m")


if __name__ == "__main__":
    main()

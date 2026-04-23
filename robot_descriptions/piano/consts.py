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

"""Piano modeling constants.

Inspired by: https://kawaius.com/wp-content/uploads/2019/04/Kawai-Upright-Piano-Regulation-Manual.pdf
"""

from dataclasses import dataclass
from math import atan

# ----- Shared constants (identical for any 88-key piano) -----

NUM_KEYS = 88
NUM_WHITE_KEYS = 52
N_SPACES_BETWEEN_WHITE_KEYS = NUM_WHITE_KEYS - 1

WHITE_KEY_INDICES = [
    0, 2, 3, 5, 7, 8, 10, 12, 14, 15, 17, 19, 20, 22, 24, 26,
    27, 29, 31, 32, 34, 36, 38, 39, 41, 43, 44, 46, 48, 50, 51, 53,
    55, 56, 58, 60, 62, 63, 65, 67, 68, 70, 72, 74, 75, 77, 79, 80,
    82, 84, 86, 87,
]
BLACK_TWIN_KEY_INDICES = [
    4, 6, 16, 18, 28, 30, 40, 42, 52, 54, 64, 66, 76, 78,
]
BLACK_TRIPLET_KEY_INDICES = [
    1, 9, 11, 13, 21, 23, 25, 33, 35, 37, 45, 47, 49, 57, 59, 61, 69, 71, 73, 81, 83, 85,
]
BLACK_KEY_INDICES = BLACK_TWIN_KEY_INDICES + BLACK_TRIPLET_KEY_INDICES

TWIN_GROUP_INDICES = list(range(2, NUM_WHITE_KEYS - 1, 7))
TRIPLET_GROUP_INDICES = list(range(5, NUM_WHITE_KEYS - 1, 7))

WHITE_KEY_COLOR = [0.9, 0.9, 0.9, 1]
BLACK_KEY_COLOR = [0.1, 0.1, 0.1, 1]
BASE_COLOR = [0.15, 0.15, 0.15, 1]

# Position-based trigger (0.70 = 70% of travel). TODO: consider velocity-based instead.
KEY_TRIGGER_THRESHOLD = 0.70


# ----- Per-variant piano parameters -----
#
# Tunable values are dataclass fields; derived values are @property, so changing
# a tunable propagates through to every geometry/dynamics value automatically.
# Units: m, rad, kg, s, N.


@dataclass(frozen=True)
class PianoParams:
    # --- Tunable parameters ---
    WHITE_KEY_WIDTH: float = 0.022
    SPACING_BETWEEN_WHITE_KEYS: float = 0.001
    WHITE_KEY_LENGTH: float = 0.15
    WHITE_KEY_TOTAL_LENGTH: float = 0.19
    BLACK_KEY_WIDTH: float = 0.01
    BLACK_KEY_LENGTH: float = 0.09
    BLACK_KEY_TOTAL_LENGTH: float = 0.12
    BLACK_KEY_HEIGHT: float = 0.018
    BLACK_OFFSET_FROM_WHITE: float = 0.0125
    BASE_HEIGHT: float = 0.180
    BASE_REAR_LENGTH: float = 0.120
    BASE_LENGTH: float = 0.295
    BASE_WIDTH: float = 1.32
    WHITE_KEY_OFFSET_FROM_BASE: float = 0.140
    WHITE_KEY_TRAVEL_DISTANCE: float = 0.012
    BLACK_KEY_TRAVEL_DISTANCE: float = 0.008
    WHITE_KEY_MASS: float = 0.04
    BLACK_KEY_MASS: float = 0.02
    KEY_SPRINGREF: float = -0.01  # rad; slight upward bias to create a restoring force
    KEY_MAX_TORQUE: float = 0.3  # Nm
    KEY_STIFFNESS: float = 4.0  # Nm/rad; must support key weight at equilibrium
    KEY_DAMPING: float = 0.05
    KEY_ARMATURE: float = 0.001

    # --- Derived values ---

    @property
    def WHITE_KEY_HEIGHT(self) -> float:
        return self.WHITE_KEY_WIDTH

    @property
    def PIANO_KEY_TOTAL_WIDTH(self) -> float:
        return (
            NUM_WHITE_KEYS * self.WHITE_KEY_WIDTH
            + N_SPACES_BETWEEN_WHITE_KEYS * self.SPACING_BETWEEN_WHITE_KEYS
        )

    @property
    def WHITE_KEY_X_OFFSET(self) -> float:
        return self.WHITE_KEY_LENGTH / 2

    # World origin is placed at the top surface of the white keys: +Z is above the
    # keybed, -Z goes down through the body.

    @property
    def WHITE_KEY_Z_OFFSET(self) -> float:
        return -self.WHITE_KEY_HEIGHT / 2

    @property
    def BLACK_KEY_X_OFFSET(self) -> float:
        return self.BLACK_KEY_LENGTH / 2

    @property
    def BLACK_KEY_Z_OFFSET(self) -> float:
        return self.BLACK_OFFSET_FROM_WHITE - self.BLACK_KEY_HEIGHT / 2

    @property
    def BASE_1_SIZE(self) -> list:
        return [self.BASE_REAR_LENGTH / 2, self.BASE_WIDTH / 2, self.BASE_HEIGHT / 2]

    @property
    def BASE_1_POS(self) -> list:
        return [
            -self.BASE_REAR_LENGTH / 2,
            0,
            self.BASE_HEIGHT / 2 - self.WHITE_KEY_OFFSET_FROM_BASE,
        ]

    @property
    def BASE_2_SIZE(self) -> list:
        return [
            self.BASE_LENGTH / 2,
            self.BASE_WIDTH / 2,
            self.WHITE_KEY_OFFSET_FROM_BASE / 2 - self.WHITE_KEY_HEIGHT / 2,
        ]

    @property
    def BASE_2_POS(self) -> list:
        return [
            -self.BASE_REAR_LENGTH + self.BASE_LENGTH / 2,
            0,
            -(self.WHITE_KEY_OFFSET_FROM_BASE + self.WHITE_KEY_HEIGHT) / 2,
        ]

    @property
    def WHITE_KEY_JOINT_MAX_ANGLE(self) -> float:
        return atan(self.WHITE_KEY_TRAVEL_DISTANCE / self.WHITE_KEY_TOTAL_LENGTH)

    @property
    def BLACK_KEY_JOINT_MAX_ANGLE(self) -> float:
        return atan(self.BLACK_KEY_TRAVEL_DISTANCE / self.BLACK_KEY_TOTAL_LENGTH)


# Portable piano: 22 mm white key width (narrower than a full-size instrument).
PIANO_PARAMS_DEP20 = PianoParams(WHITE_KEY_WIDTH=0.022)

# Full-size piano: width chosen so 52 * w + 51 * 1 mm = 1194 mm total.
PIANO_PARAMS_NIKOMAKU = PianoParams(
    WHITE_KEY_WIDTH=(1.194 - 51 * 0.001) / 52,
    WHITE_KEY_LENGTH=0.140,
    WHITE_KEY_TOTAL_LENGTH=0.170,
    BLACK_KEY_TOTAL_LENGTH=0.110,
    BASE_HEIGHT=0.085,
    BASE_REAR_LENGTH=0.060,
    BASE_LENGTH=0.210,
    BASE_WIDTH=1.210,
    WHITE_KEY_OFFSET_FROM_BASE=0.070,
)

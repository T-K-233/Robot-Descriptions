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

from math import atan

NUM_KEYS = 88
""" 88 keys on a piano. """

NUM_WHITE_KEYS = 52
""" 52 white keys on a piano. """

# White key dimensions
# WHITE_KEY_WIDTH = 0.0225  # 23.5 mm - 1 mm gap
""" Width of a white key, not including the gap. """
WHITE_KEY_WIDTH = 0.022  # 23.5 mm - 1 mm gap
# """ Portable piano key is slightly narrower  """

SPACING_BETWEEN_WHITE_KEYS = 0.001  # 1 mm
""" Space between white keys. """

WHITE_KEY_LENGTH = 0.15  # 150 mm
""" Length of the visible portion of the white key. """

WHITE_KEY_TOTAL_LENGTH = 0.19  # 190 mm
""" Total length of the white key, this is distance between the tip and the rotation axis. """

WHITE_KEY_HEIGHT = WHITE_KEY_WIDTH  # we simply set the height to be the same as the width
""" Height of the white key. """

N_SPACES_BETWEEN_WHITE_KEYS = NUM_WHITE_KEYS - 1
""" Number of gaps between white keys. """

# Black key dimensions
BLACK_KEY_WIDTH = 0.01  # 10 mm
""" Width at the top of the black key. """

BLACK_KEY_LENGTH = 0.09  # 90 mm
""" Length of the visible portion of the black key. """

BLACK_KEY_TOTAL_LENGTH = 0.12  # 120 mm
""" Total length of the black key, this is distance between the tip and the rotation axis. """

# Unlike the other dimensions, the height of the black key was roughly set such that
# when a white key is fully depressed, the bottom of the black key is barely visible.
BLACK_KEY_HEIGHT = 0.018
""" Height of the black key. """

BLACK_OFFSET_FROM_WHITE = 0.0125  # 12.5 mm
""" Offset from the top of the white key to the top of the black key. """

# Piano dimensions
PIANO_KEY_TOTAL_WIDTH = (NUM_WHITE_KEYS * WHITE_KEY_WIDTH) + (N_SPACES_BETWEEN_WHITE_KEYS * SPACING_BETWEEN_WHITE_KEYS)
""" Total width of the piano keys. """

# Base dimensions
BASE_HEIGHT = 0.180  # 180 mm
""" Height of the base. """

BASE_REAR_LENGTH = 0.120  # 120 mm
""" Length of the rear tall part of the base. """

BASE_LENGTH = 0.295  # 295 mm
""" Length of the base. """

BASE_WIDTH = 1.32  # 1320 mm
""" Width of the base. """

# Key offsets
WHITE_KEY_OFFSET_FROM_BASE = 0.140  # 140 mm
""" Offset from the bottom of the base to top of the white key. """

WHITE_KEY_X_OFFSET = WHITE_KEY_LENGTH / 2
WHITE_KEY_Z_OFFSET = WHITE_KEY_OFFSET_FROM_BASE - WHITE_KEY_HEIGHT / 2
BLACK_KEY_X_OFFSET = BLACK_KEY_LENGTH / 2
BLACK_KEY_Z_OFFSET = (WHITE_KEY_Z_OFFSET + WHITE_KEY_HEIGHT / 2) + BLACK_OFFSET_FROM_WHITE - BLACK_KEY_HEIGHT / 2

BASE_1_SIZE = [BASE_REAR_LENGTH / 2, BASE_WIDTH / 2, BASE_HEIGHT / 2]
BASE_1_POS = [-BASE_REAR_LENGTH / 2, 0, BASE_HEIGHT / 2]

BASE_2_SIZE = [BASE_LENGTH / 2, BASE_WIDTH / 2, WHITE_KEY_OFFSET_FROM_BASE / 2 - WHITE_KEY_HEIGHT / 2]
BASE_2_POS = [-BASE_REAR_LENGTH + BASE_LENGTH / 2, 0, WHITE_KEY_OFFSET_FROM_BASE / 2 - WHITE_KEY_HEIGHT / 2]

# We can write:
# tan(θ) = d / l, where d is the distance the key travels and l is the length of the
# key. Solving for θ, we get: θ = arctan(d / l).
WHITE_KEY_TRAVEL_DISTANCE = 0.012  # 12 mm
WHITE_KEY_JOINT_MAX_ANGLE = atan(WHITE_KEY_TRAVEL_DISTANCE / WHITE_KEY_TOTAL_LENGTH)
""" Maximum angle the white key joint can travel. """

# TODO(kevin): Figure out black key travel distance.
BLACK_KEY_TRAVEL_DISTANCE = 0.008  # 8 mm
""" Distance the black key travels when fully depressed. """

BLACK_KEY_JOINT_MAX_ANGLE = atan(BLACK_KEY_TRAVEL_DISTANCE / BLACK_KEY_TOTAL_LENGTH)

# Mass
WHITE_KEY_MASS = 0.04  # 40 g
BLACK_KEY_MASS = 0.02  # 20 g

# Joint spring reference, in rad
# setting the zero position to be upwards to create
# a restoring force when the key is at rest
KEY_SPRINGREF = -0.01

KEY_MAX_TORQUE = 0.3  # 0.3 Nm

# Joint spring stiffness, in Nm/rad.
# The spring should be stiff enough to support the weight of the key at equilibrium.
KEY_STIFFNESS = 4.0  # 4 Nm/rad

# Joint damping and armature for smoothing key motion.
KEY_DAMPING = 0.05
KEY_ARMATURE = 0.001

# Colors.
WHITE_KEY_COLOR = [0.9, 0.9, 0.9, 1]
BLACK_KEY_COLOR = [0.1, 0.1, 0.1, 1]
BASE_COLOR = [0.15, 0.15, 0.15, 1]

WHITE_KEY_INDICES = [
    0, 2, 3, 5, 7, 8, 10, 12, 14, 15, 17, 19, 20, 22, 24, 26, 27, 29, 31, 32, 34, 36, 38, 39, 41, 43, 44, 46, 48, 50, 51, 53, 55, 56, 58, 60, 62, 63, 65, 67, 68, 70, 72, 74, 75, 77, 79, 80, 82, 84, 86, 87,
]
""" Indices of the white keys. """

BLACK_TWIN_KEY_INDICES = [
    4, 6, 16, 18, 28, 30, 40, 42, 52, 54, 64, 66, 76, 78
]
""" Indices of the black twin keys. """

BLACK_TRIPLET_KEY_INDICES = [
    1, 9, 11, 13, 21, 23, 25, 33, 35, 37, 45, 47, 49, 57, 59, 61, 69, 71, 73, 81, 83, 85
]
""" Indices of the black triplet keys. """

BLACK_KEY_INDICES = BLACK_TWIN_KEY_INDICES + BLACK_TRIPLET_KEY_INDICES
""" Indices of the black keys. """

TWIN_GROUP_INDICES = list(range(2, NUM_WHITE_KEYS - 1, 7))

TRIPLET_GROUP_INDICES = list(range(5, NUM_WHITE_KEYS - 1, 7))

# TODO: currently we are using position as trigger, perhaps change to velocity?
KEY_TRIGGER_THRESHOLD = 0.70
""" The percentage of position travelled for the key to be considered pressed. """

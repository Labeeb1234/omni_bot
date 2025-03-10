import os
import sys
from math import pi, atan2, sin, cos, sqrt
import numpy as np
import time


def get_transform(state): # state -> [x, y, theta] 3x1 vector

    rotation = np.array([
        [cos(state[2, 0]), -sin(state[2, 0])],
        [sin(state[2, 0]), cos(state[2, 0])],
    ])
    translation = state[0:2]
    return translation, rotation

def transform_point_with_state(point, state):
    """
    Transform a point using a state.

    Args:
        point (np.array): Point [x, y] (2x1).
        state (np.array): State [x, y, theta] (3x1).

    Returns:
        np.array: Transformed point (2x1).
    """
    trans, rot = get_transform(state)
    new_point = rot @ point[0:2] + trans
    return new_point


def WrapToPi(rad, positive=False):
    """The function `WrapToPi` transforms an angle in radians to the range [-pi, pi].

    Args:

        rad (float): Angle in radians.
            The `rad` parameter in the `WrapToPi` function represents an angle in radians that you want to
        transform to the range [-π, π]. The function ensures that the angle is within this range by wrapping
        it around if it exceeds the bounds.

        positive (bool): Whether to return the positive value of the angle. Useful for angles difference.

    Returns:
        The function `WrapToPi(rad)` returns the angle `rad` wrapped to the range [-pi, pi].

    """
    while rad > pi:
        rad = rad - 2 * pi
    while rad < -pi:
        rad = rad + 2 * pi

    return rad if not positive else abs(rad)

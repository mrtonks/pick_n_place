#!/usr/bin/env python

import sys
import numpy as np
from cv2 import (
    getPerspectiveTransform,
    perspectiveTransform,
    destroyAllWindows)
from .const import (
    IMAGE_POINTS,
    OBJECT_POINTS
)

# https://stackoverflow.com/questions/36584166/how-do-i-make-perspective-transform-of-point-with-x-and-y-coordinate

def getXYPoint(u, v):
    """Returns a set of (X, Y) coordinates from real world.

    Parameters
    ----------
        u (float): Width coordinate or x
        v (float): Height coordinate or y
    
    Returns
    -------
        Array (float): [x, y] coordinates
    
    This function uses OpenCV getPerspectiveTrasnform function to calculate 
    a 3x3 perspective transform matrix. It uses four points from an image and
    the same four points from the real world to calculate the matrix. Then, 
    using the OpenCV perspectiveTrasnform function with the matrix and a 
    point (u, v) we can obtain the real world (X, Y) coordinates.
    """

    try:
        retval = getPerspectiveTransform(IMAGE_POINTS, OBJECT_POINTS)
        point = np.array([[u, v]], dtype="float32")
        point = np.array([point])  # DO NOT remove extra brackets
        p_out = perspectiveTransform(point, retval)
    except Exception as e:
        print "Error: {}".format(e)
        sys.exit()
    destroyAllWindows()
    return np.squeeze(p_out)

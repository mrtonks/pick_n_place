import numpy as np
from cv2 import solvePnP, Rodrigues
from .const import (
    IMAGE_POINTS,
    OBJECTS,
    OBJECT_POINTS,
    Z_TABLE_BAXTER
)

# https://stackoverflow.com/questions/12299870/computing-x-y-coordinate-3d-from-image-point

def getPnpPoint(u, v, object):
    retval, rvec, tvec = solvePnP(OBJECT_POINTS, IMAGE_POINTS, CAMERA_MATRIX, DIST_COEFF)  # TODO
    rotation_matrix = Rodrigues(rvec)

    uv_point = [u, v, 1]
    left_side_mat = np.linalg.inv(rotation_matrix) * np.linalg.inv(CAMERA_MATRIX) * uv_point # TODO
    right_side_mat =  np.linalg.inv(rotation_matrix) * tvec
    
    # Add the height of the object
    s = (Z_TABLE_BAXTER + OBJECTS[object] + right_side_mat) / left_side_mat

    point = np.linalg.inv(rotation_matrix) * (s * np.linalg.inv(CAMERA_MATRIX) * uv_point - tvec)
    return point
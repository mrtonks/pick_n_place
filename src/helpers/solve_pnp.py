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

# https://stackoverflow.com/questions/12299870/computing-x-y-coordinate-3d-from-image-point

# def getPnpPoint(u, v, object):
#     try:
#         retval, rvec, tvec = solvePnP(OBJECT_POINTS, IMAGE_POINTS, CAMERA_MATRIX, DIST_COEFF)
#         rvec = rvec.reshape(1, 3)
#         tvec = tvec.reshape(1, 3)
#         rotation_matrix, jacobian = Rodrigues(rvec)
#         print rotation_matrix
#         print 'Done solvePnP and Rodrigues'
#         uv_point = np.array([[u, v, 1]], dtype=np.float32)
#         uv_point = uv_point.T
#         print CAMERA_MATRIX
#         print np.linalg.inv(CAMERA_MATRIX)
#         left_side_mat = np.linalg.inv(rotation_matrix) * np.linalg.inv(CAMERA_MATRIX) * uv_point
#         print 'Done left side mat'
#         print left_side_mat
#         right_side_mat =  np.linalg.inv(rotation_matrix) * tvec
#         print 'Done right side mat'
#         print right_side_mat
        
#         # Add the height of the object
#         print 'right'        
#         s = (Z_TABLE_BAXTER + right_side_mat[0, 2]) / left_side_mat[0, 2]
#         print 'Done s'
#         print s        
#         print s * np.linalg.inv(CAMERA_MATRIX) * uv_point - tvec
#         point = np.linalg.inv(rotation_matrix) * (s * np.linalg.inv(CAMERA_MATRIX) * uv_point - tvec)
#         cv2.destroyAllWindows()
#     except Exception as e:
#         print e
#         sys.exit()
    
#     return point

#https://stackoverflow.com/questions/36584166/how-do-i-make-perspective-transform-of-point-with-x-and-y-coordinate

def getXYPoint(u, v):
    try:
        retval = getPerspectiveTransform(IMAGE_POINTS, OBJECT_POINTS)        
        point = np.array([[u,v]], dtype = "float32")
        point = np.array([point]) # It has to be done like this
        p_Out = perspectiveTransform(point, retval)        
    except Exception as e:
        print e
        sys.exit()
    destroyAllWindows()
    return np.squeeze(p_Out)
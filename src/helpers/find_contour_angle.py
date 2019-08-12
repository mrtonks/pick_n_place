#!/usr/bin/env python3

"""
This is "find_contour_angle" module.

The function "fitEllipse" will fit an ellipse into a numpy array of contour points. 
The function "fitAngle" will return an angle fitted into a maximum angle.
The function "getContourAngle" will return the angle from a contour mask.

Found in:
https://stackoverflow.com/questions/39693869/fitting-an-ellipse-to-a-set-of-data-points-in-python

Original code:
http://nicky.vanforeest.com/misc/fitEllipse/fitEllipse.html
"""

def fitAngleToBaxter(angle):
    """
    Return an angle fitted to use with Baxter. Returned angle goes from
    -179.9 to -0

    Parameters
    ----------
    angle : ```float```
        Angle to convert.

    Return
    ------
    new_angle : ```float```
        Angle transformed into Baxter's yaw.
    """ 
        
    return -90 + angle if angle <= 90 else -180 + (angle - 90)

def getContourAngle(mask, angle_type='deg'):
    """
    Return the angle of a contour.

    Parameters
    ----------
    masks : 2D array
        2D array of mask values.
    
    angle_type : String
        "rad" for radia180ns or "deg" for degrees.
    """
    import sys
    try:
        sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
    except:
        pass        
    import cv2
    import numpy as np
    from .const import ANGLE_OFFSET

    # Mask Polygon
    # Pad to ensure proper polygons for masks that touch image edges.
    padded_mask = np.zeros((mask.shape[0] + 2, mask.shape[1] + 2), dtype=np.uint8)
    padded_mask[1:-1, 1:-1] = mask
    # Helps to find the contours from the mask
    contours, _ = cv2.findContours(padded_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    # Look for the angle with major contour area
    maxArea = 0
    best = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > maxArea:
            maxArea = area
            best = contour

    ellipse = cv2.fitEllipse(best)
    (c,y),(w,h),a = ellipse
    print("Ellipse = ", a)    
    angle = 90 + a if a <= 90 else a - 90
    print("Real = ", angle)
    angle = fitAngleToBaxter(angle)
    print("Baxter = ", angle)
    if angle_type == 'rad':
        angle = np.deg2rad(angle)

    return angle

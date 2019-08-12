#!/usr/bin/env python3

"""
This is "find_contour_angle" module.

The function getContourAngle will obtain the angle from the mask sent. It will use the highest
contour area to fit an ellipse, get the angle and then it will trasnforme it into
Baxter's perspective. The function fitANgleToBaxter only transform an angle into 
Baxter's perspective. 
"""

def fitAngleToBaxter(angle):
    """
    Returns an angle fitted to use with Baxter.

    Parameters
    ----------
    angle (float): Angle to convert.

    Returns
    -------
    new_angle (float): Angle transformed into Baxter's yaw.
    """ 
    new_angle = -90.0 + angle if angle <= 90.0 else -180.0 + (angle - 90.0)
    return new_angle 

def getContourAngle(mask, angle_type="deg"):
    """
    Return the angle of a contour.

    Parameters
    ----------
    masks (array): 2D array of mask values.
    angle_type (string): Uses "rad" for radians or "deg" for degrees.
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
    
    # Transform angle from ellipses to normal perspective
    angle = 90.0 + a if a <= 90.0 else a - 90.0
    angle = fitAngleToBaxter(angle)
    if angle_type == "rad":
        # Convert degrees to radians
        angle = np.deg2rad(angle)

    return angle

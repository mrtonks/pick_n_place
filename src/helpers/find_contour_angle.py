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

def fitEllipse(cont):
    """
    Return a list with center coordinates, axis length and 
    angle of an ellipse.

    Parameters
    ----------
    cont : Numpy array 
        Array of contour points

    Returns
    ------- 
    out : List
        List with center coordinates, axis lengths and angle
    """
    import numpy as np

    x = cont[:, 0]
    y = cont[:, 1]

    x = x[:, None]
    y = y[:, None]

    D = np.hstack([x * x, x * y, y * y, x, y, np.ones(x.shape)])
    S = np.dot(D.T, D)
    C = np.zeros([6, 6])
    C[0, 2] = C[2, 0] = 2
    C[1, 1] = -1
    E, V = np.linalg.eig(np.dot(np.linalg.inv(S), C))
    n = np.argmax(np.abs(E))
    a = V[:, n]

    # Fit ellipse
    b, c, d, f, g, a = a[1] / 2., a[2], a[3] / 2., a[4] / 2., a[5], a[0]
    num = b*b-a*c
    cx = (c * d - b * f) / num
    cy = (a * f - b * d) / num

    angle = 0.5 * np.arctan(2 * b / (a - c)) * 180 / np.pi
    up = 2 * (a * f * f + c * d * d + g * b * b - 2 * b * d * f - a * c * g)
    down1 = (b * b - a * c) * ((c - a) * np.sqrt(1 + 4 * b * b / ((a - c) * (a - c))) - (c + a))
    down2 = (b * b - a * c) * ((a - c) * np.sqrt(1 + 4 * b * b / ((a - c) * (a - c))) - (c + a))
    a = np.sqrt(abs(up / down1))
    b = np.sqrt(abs(up / down2))

    params = [cx, cy, a, b, angle]
    return params

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
    return -179.9 + angle

def getContourAnglex(mask, angle_type='deg'):
    """
    Return the angle of a contour.

    Parameters
    ----------
    masks : 2D array
        2D array of mask values.
    
    angle_type : String
        "rad" for radians or "deg" for degrees.
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
    contours, _ = cv2.findContours(padded_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt = contours[0]
    # Fits an ellipse to the contours
    params = fitEllipse(cnt.squeeze())     
    print(params)
    x, y, w, h, a = params

    angle = 90 - a if (w < h) else -a
    
    print('Fitted', angle)

    if angle_type == 'rad':
        angle = np.deg2rad(angle)

    return -angle

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
    print(a)
    angle = 90 - a if a <= 90 else 180 -  a
    print("first angle: ", angle)
    angle = fitAngleToBaxter(angle)
    print("Angle for baxter = ", angle)
    if angle_type == 'rad':
        angle = np.deg2rad(angle)

    return angle
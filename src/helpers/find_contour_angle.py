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

def fitAngle(angle, max_angle=180.0):
    """
    Return a positive angle fitted to a maximum angle. If negative
    it will return the positive equivalent.

    Parameters
    ----------
    angle : ```int``` or ```float```
        Angle to convert.

    max_angle : ```int``` or ```float```
        Maximum angle to fit any angle. 
        Example: -10 is 170, -25 is 155 (max angle 180).
    """
    return angle % max_angle

def getContourAngle(mask, angle_type='deg'):
    """
    Return the angle of a contour.

    Parameters
    ----------
    masks : 2D array
        2D array of mask values.
    
    angle_type : String
        "rad" for radians or "deg" for degrees.
    """
    import cv2
    import numpy as np

    # Mask Polygon
    # Pad to ensure proper polygons for masks that touch image edges.
    padded_mask = np.zeros((mask.shape[0] + 2, mask.shape[1] + 2), dtype=np.uint8)
    padded_mask[1:-1, 1:-1] = mask
    # Helps to find the contours from the mask
    contours, _ = cv2.findContours(padded_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt = contours[0]
    # Fits an ellipse to the contours
    params = fitEllipse(cnt.squeeze())
    # In case of negative angle, get positive equivalent
    angle = fitAngle(params[4])

    if angle_type == 'rad':
        angle = np.deg2rad(angle)

    return angle
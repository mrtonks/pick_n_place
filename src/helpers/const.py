import numpy as np
from geometry_msgs.msg import Quaternion

# --------------------------------------
# General constants
# --------------------------------------
# Classes as contained in the coco definitions file.
# Same order (id number) as in the coco file. 
# If more classes are added, they need to be included here.
# BG = background
CLASSES = [
    'BG', 'cat_cup', 'black_trainer', 'small_tupper',
    'katana_umbrella', 'harrogate_water', 'feet_spray',
    'highland_water', 'catbus', 'snapback_hat', 'unstable_unicorns'
]
LIMB = 'right'

# --------------------------------------
# Measurements (meters)
# --------------------------------------

HOVER_DISTANCE = 0.15  # meters
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
# Object heigth in meters
OBJECTS = {
    'black_trainer': 0.08,
    'catbus': 0.06,
    'cat_cup': 0.10,
    'feet_spray': 0.04,
    'harrogate_water': 0.04,
    'highland_water': 0.04,
    'katana_umbrella': 0.05,
    'small_tupper': 0.045,
    'snapback_hat': 0.10,
    'unstable_unicorns': 0.1
}
Z_TABLE_BAXTER = -0.21  # Could be different, but grip hits table on -0.21 m
Z_GRIP_DEPTH = 0.04  # 4 cms for the grip depth

# --------------------------------------
# Baxter movements
# --------------------------------------
# This quaternions work great. (176, 0, -179)
# Quaternions for placing the object
OVERHEAD_ORIENTATION = Quaternion(
    x=0.00786117417644,
    y=0.999878111508,
    z=-0.0125115456827,
    w=-0.00504234997407,
)
# Starting position from grip
START_JOINT_ANGLES = {
    'right_w0': -0.4843544337748194,
    'right_w1': 1.3694613483847031,
    'right_w2': -0.044485442848677,
    'right_e0': 1.3602574636573905,
    'right_e1': 1.7303303287347467,
    'right_s0': -0.7558690332305377,
    'right_s1': -1.153553552489831
}
Y_PLACING = 0.25  # Distance to move the object

# --------------------------------------
# Camera Calibration
# --------------------------------------
CAMERA_MATRIX = np.array([
    (723.044309, 0.000000, 472.782029),
    (0.000000, 705.863044, 310.658635),
    (0.000000, 0.000000, 1.000000)
], dtype=np.float32)
DIST_COEFF = np.array([-0.145259, -0.035861, -0.009946, -0.033134, 0.000000], dtype=np.float32)
# Table's corners' coordinates from the image in pixels
# Calibrate manually if camera is moved, from right camera
# Run object_detection.py alone
IMAGE_POINTS = np.array([
    (407.677, 271.147),  # Upper left (x, y)
    (322.129, 626.097),  # Lower left
    (978.0, 285.581),  # Upper right
    (1050.13, 637.839)  # Lower right
], dtype=np.float32)
# Table's corners' coordinates from Baxter's perspective
# Calibrate manually if baxter or table is moved
# Use "rostopic echo -n 1 /robot/limb/{limb}/endpoint_state"
OBJECT_POINTS = np.array([
    (0.972906622465, 0.513606512261),  # Upper left (X, Y)
    (0.346150216655, 0.491463406542),  # Lower left
    (0.996763251255, -0.512213883443),  # Upper right
    (0.372487505071, -0.513767038756)  # Lower right
], dtype=np.float32)

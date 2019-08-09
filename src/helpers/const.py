from geometry_msgs.msg import Quaternion

ANGLE_OFFSET = 90  # Degrees
# Classes as contained in the coco definitions file
# Same order (id number) as in the coco file
# BG = background
CLASSES = [
    'BG', 'cat_cup', 'black_trainer', 'small_tupper',
    'katana_umbrella', 'harrogate_water', 'feet_spray',
    'highland_water', 'catbus', 'snapback_hat', 'unstable_unicorns'
]
HOVER_DISTANCE = 0.15  # meters
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
LIMB = 'right'
# Object heigth in meters
OBJECTS = {
    'black_trainer': 0.08,
    'catbus': 0.05,
    'cat_cup': 0.10,  # TODO
    'feet_spray': 0.04,
    'harrogate_water': 0.04,
    'highland_water': 0.04,
    'katana_umbrella': 0.05,
    'small_tupper': 0.045,
    'snapback_hat': 0.10,  # TODO
    'unstable_unicorns': 0.1
}
# This quaternions work great. (176, 0, -179)
OVERHEAD_ORIENTATION = Quaternion(
    x=0.00786117417644,
    y=0.999878111508,
    z=-0.0125115456827,
    w=-0.00504234997407,
)
OVERHEAD_ORIENTATION_START = Quaternion(
    x=-0.00337619259824,
    y=0.99983167818,
    z=-0.0166998965825,
    w=0.00680662077238
)
OVERHEAD_ORIENTATION_ANGLES = [176, 0, 177]
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
# Table's corners' coordinates from Baxter's perspective
# Calibrate manually if baxter or table is moved
# Use "rostopic echo -n 1 /robot/limb/{limb}/endpoint_state"
TABLE_BAXTER = {
    'upper_left': {'x': 1.00539650803, 'y': 0.515422346073},
    'lower_left': {'x': 0.385302617015, 'y': 0.474767834085},
    'upper_right': {'x': 1.06705514594, 'y': -0.507285249659},
    'lower_right': {'x':  0.441618955699, 'y': -0.520428725454}
}
# Table's corners' coordinates from the image in pixels
# Calibrate manually if camera is moved, from right camera
# Run object_detection.py alone
TABLE_IMAGE = {
    'upper_left': {'x': 419.419, 'y': 305.995},
    'lower_left': {'x': 342.258, 'y': 649.724},
    'upper_right': {'x': 972.968, 'y': 299.286},
    'lower_right': {'x': 1066.9, 'y': 656.494}
}
X_OFFSET = 0.00  # Roughly estimation (meters)
Y_OFFSET = 0.06
Y_OFFSET_RIGHT = 0.0
Y_PLACING = 0.25
Z_TABLE_BAXTER = -0.20  # Could be different, but grip hits table on -0.20
Z_GRIP_DEPTH = 0.04  # 4 cms for the grip depth

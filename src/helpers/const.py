from geometry_msgs.msg import Quaternion

# Classes as contained in the coco definitions file
# Same order (id number) as in the coco file
# BG = background
CLASSES = [
    'BG', 'cat_cup', 'black_trainer', 'small_tupper',
    'katana_umbrella', 'harr4ogate_water', 'feet_spray',
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
# Use "rostopic echo -n 1 /robot/"limb"/endpoint_state"
TABLE_BAXTER = {
    'upper_left': {'x': 1.08376362735, 'y': 0.467429998229},
    'lower_left': {'x': 0.444348149295, 'y': 0.477504802962},
    'upper_right': {'x': 1.01795889999, 'y': -0.562818766745},
    'lower_right': {'x':  0.398211437682, 'y': -0.520174310791}
}
# Table's corners' coordinates from the image in pixels
# Calibrate manually if camera is moved, from right camera
TABLE_IMAGE = {
    'upper_left': {'x': 360.71, 'y': 300.686},
    'lower_left': {'x': 276.839, 'y': 644.557},
    'upper_right': {'x': 914.258, 'y': 302.364},
    'lower_right': {'x': 999.806, 'y': 637.848}
}
X_OFFSET = 0.0  # Roughly estimation (meters)
Y_OFFSET = 0.05
Y_PLACING = 0.25
Z_TABLE_BAXTER = -0.20  # Could be different, but grip hits table on -0.20
Z_GRIP_DEPTH = 0.04  # 4 cms for the grip depth

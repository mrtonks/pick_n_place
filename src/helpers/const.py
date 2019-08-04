# Classes as contained in the coco definitions file
# Same order (id number) as in the coco file
# BG = background
CLASSES = [ 
    'BG', 'cat_cup', 'black_trainer', 'small_tupper',
    'katana_umbrella', 'harrogate_water', 'feet_spray',
    'highland_water', 'catbus', 'snapback_hat', 'unstable_unicorns'
]
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
# Object heigth in meters
OBJECTS = {
    'black_trainer': 0.10,
    'highland_water': 0.04,
    'harrogate_water': 0.04,
    'katana_umbrella': 0.05,
    'small_tupper': 0.045,
}

# Table's corners' coordinates from Baxter's perspective
# Calibrate manually if baxter or table is moved
# Use "rostopic echo -n 1 /robot/"limb"/endpoint_state"
TABLE_BAXTER = {
    'upper_left': {'x': 1.100307662, 'y': 0.446151068025},
    'lower_left': {'x': 0.488212791458, 'y': 0.487833555787},
    'upper_right': {'x': 1.07260257605, 'y': -0.560748453554},
    'lower_right': {'x':  0.436149979162, 'y': -0.515481924476}
}
# Table's corners' coordinates from the image in pixels
# Calibrate manually if camera is moved, from right camera
TABLE_IMAGE = {
    'upper_left': {'x': 397.613, 'y': 244.308},
    'lower_left': {'x': 300.323, 'y': 563.018},
    'upper_right': {'x': 939.419, 'y': 266.115},
    'lower_right': {'x': 1001.48, 'y': 596.566}
}
X_OFFSET = 0.03 # Can be changed according to needs. Roughly estimation (meters)
Y_OFFSET = 0.03 
Z_TABLE_BAXTER = -0.20 # Could be different, but grip hits table on -0.20
Z_GRIP_DEPTH = 0.04 # 4 cms for the grip depth

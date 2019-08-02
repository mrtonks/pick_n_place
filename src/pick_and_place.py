#!/usr/bin/env python

import sys
import cv2
import rospy
import zmq
import pickle
import json
import math
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from zmq.eventloop import ioloop, zmqstream

import move_arm

# Constants
OBJECTS = {
    'black_trainer': 0.10,
    'highland_water': 0.04,
    'harrogate_water': 0.04,
    'katana_umbrella': 0.05,
    'small_tupper': 0.045,
}

TABLE_BAXTER = {
    'upper_left': {'x': 1.100307662, 'y': 0.446151068025},
    'lower_left': {'x': 0.488212791458, 'y': 0.487833555787},
    'upper_right': {'x': 1.07260257605, 'y': -0.560748453554},
    'lower_right': {'x':  0.436149979162, 'y': -0.515481924476}
}
# Calibrate manually if camera is moved
# From right camera
TABLE_IMAGE = {
    'upper_left': {'x': 397.613, 'y': 244.308},
    'lower_left': {'x': 300.323, 'y': 563.018},
    'upper_right': {'x': 939.419, 'y': 266.115},
    'lower_right': {'x': 1001.48, 'y': 596.566}
}
Z_TABLE_BAXTER = -0.20 # obj_detectedCould be different, but grip hits on -0.20
Z_GRIP_DEPTH = 0.04 # 4 cms for the grip depth
X_OFFSET = 0.03
Y_OFFSET = 0.03

# Global variables
bridge = None
objects_detected = None
limb = 'right'


def calculateObjPose(obj_to_pick, u, v):
    """ Calculate the object pose.

    Params:
    obj_to_pick = Name of the object to pick and move,
    u = height in pixels (x coordinate wrt Baxter),
    v = width in pixels (y coordinate wrt Baxter),

    Return:
    x_baxter = Coordinate x wrt Baxter,
    y_baxter = Coordinate y wrt Baxter,
    """

    x_baxter = 0
    y_baxter = 0
    z_baxter = 0

    if obj_to_pick is None:
        print 'No object to pick. Check objects_detected.'
        return
    elif obj_to_pick not in OBJECTS:
        print 'The object to pick is not registered. Check OBJECTS.'
        return
    else:
        z_baxter = Z_TABLE_BAXTER + OBJECTS[obj_to_pick] - Z_GRIP_DEPTH    

    # Obtain table width wrt Baxter (should be meters?)
    #table_width_bx = abs(TABLE_BAXTER['lower_left']['y']) + abs(TABLE_BAXTER['lower_right']['y'])
    # Obtain table width wrt image
    #table_width_px = TABLE_IMAGE['lower_right']['x'] - TABLE_IMAGE['lower_left']['x']
    # Obtain v wrt to the table in the image
    #v_wrt_table_px = v - TABLE_IMAGE['lower_left']['x']
    image_width_ctr = 1280 / 2        
    if v < image_width_ctr:
        image_fixed = image_width_ctr - TABLE_IMAGE['lower_left']['x']
        v_fixed = image_width_ctr - v
        y_baxter = ((v_fixed * TABLE_BAXTER['lower_left']['y']) / image_fixed) - Y_OFFSET
        #- TABLE_BAXTER['lower_left']['y'] 
    else:
        image_fixed = TABLE_IMAGE['lower_right']['x'] - image_width_ctr 
        v_fixed = v - image_width_ctr
        y_baxter = ((v_fixed * TABLE_BAXTER['lower_right']['y']) / image_fixed) - Y_OFFSET 
        #- TABLE_BAXTER['lower_left']['y'] #+ Y_OFFSET
        
    # Apply three simple rule and substract from y positive value
    
    # y_baxter = (v_wrt_table_px * table_width_bx / table_width_px) \
    #     - TABLE_BAXTER['lower_left']['y']

    # Obtain table height wrt Baxter (should be meters?)
    table_height_bx = TABLE_BAXTER['upper_left']['x'] #- TABLE_BAXTER['lower_left']['x']
    # Obtain table height wrt image
    table_height_px = TABLE_IMAGE['upper_left']['y']
    #table_height_px = TABLE_IMAGE['lower_left']['y'] - TABLE_IMAGE['upper_left']['y']
    # Obtain u wrt to the table in the image
    #u_wrt_table_px = u - TABLE_IMAGE['upper_left']['y']
    # Apply three simple rule and substract from y positive value
    x_baxter = ((table_height_bx * table_height_px) / u) - X_OFFSET
    #x_baxter = (u_wrt_table_px * table_height_bx * table_height_px)

    return x_baxter, y_baxter, z_baxter


def receiveObjectsDetected(data):
    global objects_detected

    print 'Receiving objects detected...'
    try:
        objects_detected = pickle.loads(data[0])
    except pickle.PickleError as e:
        print 'Error: {}'.format(e)
        sys.exit(1)

def moveObject(data):
    global bridge
    global objects_detected
    global limb

    if objects_detected is None:
        return

    try:
        depth_image = bridge.imgmsg_to_cv2(data, "32FC1")
    except CvBridgeError as e:
        print 'Error: {}'.format(e)

    depth_array = np.array(depth_image, dtype=np.float32)
    print('Image size: {width}x{height}'.format(width=data.width, height=data.height))
            
    count_obj_detected = len(objects_detected)
    #print 'Number of objects detected: {}'.format(count_obj_detected)
    # Create numpy arrays for distances and names
    obj_distances = np.empty((count_obj_detected))
    obj_names = np.empty((count_obj_detected), dtype="S20")
    obj_u = np.empty_like(obj_distances)
    obj_v = np.empty_like(obj_distances)
    x_px = np.empty_like(obj_distances)
    for obj_idx in range(count_obj_detected):
        obj_detected = objects_detected[str(obj_idx)]
        name = obj_detected['name']
        coordinates = obj_detected['coordinates']       
        print coordinates       
        # Calculate u (height) and v (width)
        # Coordinates = [y1, x1, y2, x2]
        u = ((coordinates[2] - coordinates[0]) / 2) + coordinates[0]
        v = ((coordinates[3] - coordinates[1]) / 2) + coordinates[1]
        # Obtain distance
        dist = depth_array[u, v] # 720x1280               
        obj_distances[obj_idx] = dist
        obj_names[obj_idx] = name
        obj_u[obj_idx] = u
        obj_v[obj_idx] = v  
        x_px[obj_idx] = coordinates[1]
    # Get index from closet object
    closest_obj = np.argmin(obj_distances)  
    print 'Objects detected: {}'.format(', '.join(obj_names))          
    print 'Closest object: {} - {} m\n\n'.format(obj_names[closest_obj], obj_distances[closest_obj])            
    #getDistanceFromCenter(data, depth_array, obj_u[closest_obj], obj_v[closest_obj])
    if 0.3 < obj_distances[closest_obj] < 1.5:
        x, y, z = calculateObjPose(obj_names[closest_obj], obj_u[closest_obj], obj_v[closest_obj])
        # Publish that Baxter is about to move
        is_moving_pub.publish(True)
        move_arm.initplannode([x, y, z], limb)        
        
    is_moving_pub.publish(False)
    objects_detected = None

def subscriberObjectDetection():
    addr = '127.0.0.1'  # remote ip or localhost
    port = '5556'  # same as in the pupil remote gui
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt_string(zmq.SUBSCRIBE, u'')
    print 'Collecting updates...'
    socket.connect('tcp://{}:{}'.format(addr, port))
    print 'Waiting...'
    stream = zmqstream.ZMQStream(socket)
    stream.on_recv(receiveObjectsDetected)
    try:
           ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:
        print('\nListener has stopped')     


if __name__ == '__main__':
    rospy.init_node('pick_and_place', log_level=rospy.INFO)
    bridge = CvBridge()
    
    is_moving_pub = rospy.Publisher("is_moving", Bool, queue_size=10)
    rospy.Subscriber('/zed/zed_node/depth/depth_registered', Image, moveObject)
    subscriberObjectDetection()     
    is_moving_pub.publish(False)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('distance_calculation node terminated')
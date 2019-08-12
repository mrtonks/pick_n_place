#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Pick and Place
The pick and place main processes.

Copyright (c) 2019 Jesus Vera
Licensed under the MIT License (see LICENSE for details)
Written by Jesus Vera
"""

import sys
import json
import math

import zmq
import pickle
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from zmq.eventloop import ioloop, zmqstream

# ROS imports
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler

from helpers import const, move_arm, solve_perspective

# Global variables
bridge = None

def calculateObjPose(obj, u, v, orientation):
    """
    Calculate the object pose.

    Parameters
    ----------
        obj (str): Name of the object to pick and move
        u (float): Width in ``pixels`` (y coordinate wrt Baxter)
        v (float): Height in ``pixels`` (x coordinate wrt Baxter)

    Returns
    -------
        x_baxter (float): Coordinate x wrt Baxter.
        y_baxter (float): Coordinate y wrt Baxter.
        z_baxter (float): Coordinate z wrt Baxter.
        quaternions (float array): Array contains [x, y, z, w] values.
    """

    x_baxter = 0
    y_baxter = 0
    z_baxter = 0

    # If the object comes empty, or it does not
    # exists in the constant OBJECTS, return.
    # Else, calculate coordinate Z wrt Baxter.
    if obj is None:
        print 'No object to pick. Check objects_detected.'
        return
    elif obj not in const.OBJECTS:
        print 'The object to pick is not registered. Check OBJECTS.'
        return
    else:
        z_baxter = const.Z_TABLE_BAXTER + const.OBJECTS[obj] - const.Z_GRIP_DEPTH    
        
    # Get XY point coordinates from real world coordinates
    x_baxter, y_baxter = solve_perspective.getXYPoint(u, v)    
    quaternions = quaternion_from_euler(np.deg2rad(176.0), 0.0, orientation)  # Radians
    if quaternions[1] < 0:
        # The y quaternion comes negative and it must be positive 
        # or the gripper will not use the correct angle value
        quaternions[1] = -quaternions[1]
    return x_baxter, y_baxter, z_baxter, quaternions

def receiveObjectsDetected(data):
    """
    Receive the objects detected.

    The objects detected are stored in the global variable objects_detected. 
    Then it will call the function moveObject and it will send the objects
    
    Parameters
    ----------
        data (pickle file): Pickle object containing objects detected. 
    """

    print "Receiving objects detected..."
    try:
        objects_detected = pickle.loads(data[0])        
    except pickle.PickleError as e:
        rospy.logerr('Error: {}'.format(e))

    if objects_detected:
        image_depth = rospy.wait_for_message('/zed/zed_node/depth/depth_registered', Image)
        moveObject(objects_detected, image_depth)
    

def moveObject(objects_detected, image):
    """Extract values from objects_detected object and moves arm.

    The function will extarct the object values from the objects_detected and 
    it will call the function that moves the arm sending the values from the
    closest object.
    
    Parameters
    ----------
        objects_detected (object): Objects detected containing name, coordinates and orientation.
        image (sensor_msgs.msg.Image): Depth image.
    """
    global bridge

    if objects_detected is None:
        return

    try:
        image_cv2 = bridge.imgmsg_to_cv2(image, "32FC1")
    except CvBridgeError as e:
        print "Error: {}".format(e)

    depth_array = np.array(image_cv2, dtype=np.float32)
    print "Image size: {}x{}".format(image.width, image.height)
    count_obj_detected = len(objects_detected)
    # Create numpy arrays for distances and names
    # obj_distances = np.zeros((count_obj_detected))
    obj_names = np.empty((count_obj_detected), dtype="S20")
    obj_values = np.zeros((count_obj_detected, 4), dtype=np.float)  # [distance, x, y, orientation]
    # obj_orientation = np.zeros((count_obj_detected))
    for obj_idx in range(count_obj_detected):
        obj_detected = objects_detected[str(obj_idx)]
        obj_names[obj_idx] = obj_detected['name']
        coordinates = obj_detected['coordinates']
        # Calculate u (width) and v (height)
        # Coordinates = [y1, x1, y2, x2]
        u = ((coordinates[3] - coordinates[1]) / 2) + coordinates[1] # x - width
        v = ((coordinates[2] - coordinates[0]) / 2) + coordinates[0] # y - height        
        dist = depth_array[v, u]  # Obtain depth distance, 720x1280
        obj_values[obj_idx, 0] = dist
        obj_values[obj_idx, 1] = u  # x
        obj_values[obj_idx, 2] = v  # y
        obj_values[obj_idx, 3] = obj_detected['orientation']
    closest_obj = np.argmin(obj_values[:, 0]) # Get index from closet object  
    print "Objects detected: {}".format(', '.join(obj_names))          
    print "Closest object: {} - {} m\n".format(obj_names[closest_obj], obj_values[closest_obj, 0])     
    print "Pose: x: {}, y: {}, angle: {}".format(obj_values[closest_obj, 1], \
        obj_values[closest_obj, 2], obj_values[closest_obj, 3])      
    
    if 0.8 < obj_values[closest_obj, 0] < 1.3:
        # Check if object is less than 0.5 m closer or more 1.5 m further
        x, y, z, quaternions = calculateObjPose(obj_names[closest_obj], obj_values[closest_obj, 1], \
            obj_values[closest_obj, 2], obj_values[closest_obj, 3])
        if x <> 0 or y <> 0 or z <> 0:            
            is_moving_pub.publish(True)  # Publish that Baxter is about to move
            move_arm.initplannode([x, y, z], quaternions, const.LIMB)  # Start moving arm 
    else:
        print "Closest object is out of pick up distance."              
    
    is_moving_pub.publish(False)  # Publish that Baxter is not moving anymore

def subscriberObjectDetection():
    """Subscriber for Object Detection message using ZMQ"""

    addr = "127.0.0.1"  # remote ip or localhost
    port = "5556"
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt_string(zmq.SUBSCRIBE, u'')
    print "Collecting updates..."
    socket.connect('tcp://{}:{}'.format(addr, port))
    print "Waiting..."
    stream = zmqstream.ZMQStream(socket)
    stream.on_recv(receiveObjectsDetected)
    try:
        ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:
        print "Listener has stopped"     


if __name__ == "__main__":
    rospy.init_node('pick_and_place', log_level=rospy.INFO)
    bridge = CvBridge()
    
    try:
        is_moving_pub = rospy.Publisher("is_moving", Bool, queue_size=10)
        subscriberObjectDetection()     
        is_moving_pub.publish(False)
        rospy.spin()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        rospy.loginfo('pick_and_place node terminated')
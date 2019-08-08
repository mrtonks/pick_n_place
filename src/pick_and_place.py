#!/usr/bin/env python

import sys
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
from tf.transformations import quaternion_from_euler
from helpers import const
from helpers import move_arm

# Global variables
bridge = None
objects_detected = None


def calculateObjPose(obj_to_pick, u, v, orientation):
    """
    Calculate the object pose.

    Params:
    :param obj_to_pick: Name of the object to pick and move
    :param u: height in ``pixels`` (x coordinate wrt Baxter)
    :param v: width in ``pixels`` (y coordinate wrt Baxter)

    Returns:
    x_baxter = Coordinate x wrt Baxter,
    y_baxter = Coordinate y wrt Baxter,
    z_baxter = Coordinate z wrt Baxter.
    """

    x_baxter = 0
    y_baxter = 0
    z_baxter = 0

    if obj_to_pick is None:
        print 'No object to pick. Check objects_detected.'
        return 0, 0, 0
    elif obj_to_pick not in const.OBJECTS:
        print 'The object to pick is not registered. Check OBJECTS.'
        return 0, 0, 0
    else:
        z_baxter = const.Z_TABLE_BAXTER + const.OBJECTS[obj_to_pick] - const.Z_GRIP_DEPTH    

    # Use the width middle point to get y = 0 (coordinate)
    # and determine the side of the object according to y (left or right / + or -)
    image_width_mid = const.IMAGE_WIDTH / 2        
    if v < image_width_mid:
        image_fixed = image_width_mid - const.TABLE_IMAGE['lower_left']['x']
        v_fixed = image_width_mid - v
        # Apply three simple rule to calculate the y coordinate wrt Baxter
        y_baxter = ((v_fixed * const.TABLE_BAXTER['lower_left']['y']) / image_fixed) - const.Y_OFFSET
    else:
        image_fixed = const.TABLE_IMAGE['lower_right']['x'] - image_width_mid
        v_fixed = v - image_width_mid
        y_baxter = ((v_fixed * const.TABLE_BAXTER['lower_right']['y']) / image_fixed) - const.Y_OFFSET 

    table_height_bx = const.TABLE_BAXTER['upper_left']['x'] # Obtain table height wrt Baxter
    table_height_px = const.TABLE_IMAGE['upper_left']['y'] # Obtain table height wrt image
    x_baxter = ((table_height_bx * table_height_px) / u) - const.X_OFFSET # Apply three simple rule 

    # Obtain the orientation
    quaternions = quaternion_from_euler(176, 0, -orientation)
    print 'First quaternions, ', quaternions
    return x_baxter, y_baxter, z_baxter, quaternions


def receiveObjectsDetected(data):
    """
    Receive the objects detected
    
    Params:
    :param data: ``Image`` data type from ``sensor_msgs`` 
    """
    global objects_detected

    print 'Receiving objects detected...'
    try:
        objects_detected = pickle.loads(data[0])        
    except pickle.PickleError as e:
        rospy.logerr('Error: {}'.format(e))
    if objects_detected:
        image_depth = rospy.wait_for_message('/zed/zed_node/depth/depth_registered', Image)
        moveObject(objects_detected, image_depth)
    

def moveObject(obj_detected, image):
    global bridge
    global objects_detected

    if objects_detected is None:
        return

    try:
        image_cv2 = bridge.imgmsg_to_cv2(image, "32FC1")
    except CvBridgeError as e:
        print 'Error: {}'.format(e)

    depth_array = np.array(image_cv2, dtype=np.float32)
    print 'Image size: {}x{}'.format(image.width, image.height)
    count_obj_detected = len(objects_detected)
    # Create numpy arrays for distances and names
    obj_distances = np.zeros((count_obj_detected))
    obj_names = np.empty((count_obj_detected), dtype="S20")
    obj_coordinates = np.zeros((count_obj_detected, 2))
    obj_orientation = np.zeros((count_obj_detected))
    #obj_v = np.empty_like(obj_distances)
    for obj_idx in range(count_obj_detected):
        obj_detected = objects_detected[str(obj_idx)]
        name = obj_detected['name']
        coordinates = obj_detected['coordinates']
        # print coordinates       
        # Calculate u (height) and v (width)
        # Coordinates = [y1, x1, y2, x2]
        u = ((coordinates[2] - coordinates[0]) / 2) + coordinates[0] # y - height
        v = ((coordinates[3] - coordinates[1]) / 2) + coordinates[1] # x - width
        dist = depth_array[u, v] # Obtain distance, 720x1280
        obj_distances[obj_idx] = dist
        obj_names[obj_idx] = name
        obj_coordinates[obj_idx, 0] = u
        obj_coordinates[obj_idx, 1] = v
        obj_orientation[obj_idx] = obj_detected['orientation']
    closest_obj = np.argmin(obj_distances) # Get index from closet object  
    print 'Objects detected: {}'.format(', '.join(obj_names))          
    print 'Closest object: {} - {} m\n'.format(obj_names[closest_obj], obj_distances[closest_obj])     
    print 'Pose: x: {}, y: {}, angle: {}'.format(obj_coordinates[closest_obj, 0], \
        obj_coordinates[closest_obj, 1], np.rad2deg(obj_orientation[closest_obj]))       
    # Check if object is less than 0.5 m closer or more 1.5 m further
    if 0.5 < obj_distances[closest_obj] < 1.5:
        x, y, z, quaternions = calculateObjPose(obj_names[closest_obj], obj_coordinates[closest_obj, 0], \
            obj_coordinates[closest_obj, 1], obj_orientation[closest_obj])
        if x <> 0 or y <> 0 or z <> 0:
            print 'Orientation ', obj_orientation[closest_obj]
            print 'Quaterions ', quaternions
            is_moving_pub.publish(True) # Publish that Baxter is about to move
            move_arm.initplannode([x, y, z], quaternions, const.LIMB) # Start moving  
    else:
        print 'Closest object is out of pick up distance.'              
    
    is_moving_pub.publish(False) # Publish that Baxter is not moving anymore
    objects_detected = None

def subscriberObjectDetection():
    """Subscriber for Object Detection message using ZMQ"""
    addr = '127.0.0.1'  # remote ip or localhost
    port = '5556' 
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
        print('Listener has stopped')     


if __name__ == '__main__':
    rospy.init_node('pick_and_place', log_level=rospy.INFO)
    bridge = CvBridge()
    
    try:
        is_moving_pub = rospy.Publisher("is_moving", Bool, queue_size=10)
        #rospy.Subscriber('/zed/zed_node/depth/depth_registered', Image, moveObject)
        subscriberObjectDetection()     
        is_moving_pub.publish(False)
        rospy.spin()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        rospy.loginfo('pick_and_place node terminated')
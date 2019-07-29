#!/usr/bin/env python

import sys
import cv2
import rospy
import zmq
import pickle
import json
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from zmq.eventloop import ioloop, zmqstream

bridge = None
objects_detected = None

def receiveObjectsDetected(data):
    global objects_detected

    print 'Receiving objects detected...'
    try:
        objects_detected = pickle.loads(data[0])
    except pickle.PickleError as e:
        print 'Error: {}'.format(e)
        sys.exit(1)

def getImageDepth(data):
    global bridge
    global objects_detected

    if objects_detected is None:
        return

    try:
        depth_image = bridge.imgmsg_to_cv2(data, "32FC1")
    except CvBridgeError as e:
        print 'Error: {}'.format(e)

    depth_array = np.array(depth_image, dtype=np.float32)
    print('Image size: {width}x{height}'.format(width=data.width,height=data.height))
            
    try:
        count_obj_detected = len(objects_detected)
        #print 'Number of objects detected: {}'.format(count_obj_detected)
        # Create numpy arrays for distances and names
        obj_distances = np.empty((count_obj_detected))
        obj_names = np.empty((count_obj_detected), dtype="S20")
        for obj_idx in range(count_obj_detected):
            obj_detected = objects_detected[str(obj_idx)]
            name = obj_detected['name']
            coordinates = obj_detected['coordinates']                
            # Calculate u (height) and v (width)
            # Coordinates = [y1, x1, y2, x2]
            u = ((coordinates[2] - coordinates[0]) / 2) + coordinates[0]
            v = ((coordinates[3] - coordinates[1]) / 2) + coordinates[1]
            # Obtain distance
            dist = depth_array[u, v]                
            obj_distances[obj_idx] = dist
            obj_names[obj_idx] = name            
        # Get index from closet object
        closest_obj = np.argmin(obj_distances)  
        print 'Objects detected: {}'.format(', '.join(obj_names))          
        print 'Closest object: {} - {} m'.format(obj_names[closest_obj], obj_distances[closest_obj])            
    except Exception as e:
        print(e)
        
    objects_detected = None

def main():
    global bridge
    global objects_detected
    
    rospy.init_node('distance_calculation', log_level=rospy.INFO)
    bridge = CvBridge()
    rospy.Subscriber('/zed/zed_node/depth/depth_registered', Image, getImageDepth)

    addr = '127.0.0.1'  # remote ip or localhost
    port = '5556'  # same as in the pupil remote gui
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt_string(zmq.SUBSCRIBE, u'')
    print 'Collecting updates...'
    socket.connect('tcp://{}:{}'.format(addr, port))
    print 'Waiting...'
    stream = zmqstream.ZMQStream(socket)
    while True:
        stream.on_recv(receiveObjectsDetected)        
    #rate = rospy.Rate(50)
    #while objects_detected is None:
    #    rate.sleep()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('distance_calculation node terminated')

if __name__ == '__main__':
    main()
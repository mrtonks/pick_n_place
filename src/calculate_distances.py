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
from cv_bridge import CvBridge, CvBridgeError
from zmq.eventloop import ioloop, zmqstream

#import moveit_commander
#import moveit_msgs.msg
#import geometry_msgs.msg


bridge = None
objects_detected = None
small_tupper = 0.045

def getDistanceFromCenter(data, depth_array, u, v):
    cam_to_table = 1.04
    
    v_middle_width = data.width / 2
    print data.width, v_middle_width
    dist_center = depth_array[u, v_middle_width]
    mid_point_to_center = math.sqrt(dist_center**2 - cam_to_table**2)
    
    dist_object = depth_array[u, v]
    cam_to_object = cam_to_table - small_tupper
    obj_to_center = math.sqrt(dist_object**2 - cam_to_object**2)
    print 'Mid point to center - ', mid_point_to_center
    
    dist_center_to_object = math.sqrt(obj_to_center**2 - mid_point_to_center**2)
    print 'Distance object to center: {}'.format(dist_center_to_object)
    
    ## try to move baxter
    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # group = moveit_commander.MoveGroupCommander("left_arm")
    # display_trajectory_publisher = rospy.Publisher(
    #                                 '/move_group/display_planned_path',
    #                                 moveit_msgs.msg.DisplayTrajectory)
    # print "============ Waiting for RVIZ..."
    # rospy.sleep(10)
    # print "============ Starting tutorial "
    

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
    print('Image size: {width}x{height}'.format(width=data.width, height=data.height))
            
    count_obj_detected = len(objects_detected)
    #print 'Number of objects detected: {}'.format(count_obj_detected)
    # Create numpy arrays for distances and names
    obj_distances = np.empty((count_obj_detected))
    obj_names = np.empty((count_obj_detected), dtype="S20")
    obj_u = np.empty_like(obj_distances)
    obj_v = np.empty_like(obj_distances)
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
        obj_u[obj_idx] = u
        obj_v[obj_idx] = v            
    # Get index from closet object
    closest_obj = np.argmin(obj_distances)  
    print 'Objects detected: {}'.format(', '.join(obj_names))          
    print 'Closest object: {} - {} m\n\n'.format(obj_names[closest_obj], obj_distances[closest_obj])            
    #getDistanceFromCenter(data, depth_array, obj_u[closest_obj], obj_v[closest_obj])
    
    
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


def main():
    global bridge
    global objects_detected
    
    rospy.init_node('distance_calculation', log_level=rospy.INFO)
    bridge = CvBridge()
    try:
        main()
    except KeyboardInterrupt:
        rospy.signal_shutdown('ROS stopped')    
    rospy.Subscriber('/zed/zed_node/depth/depth_registered', Image, getImageDepth)
    subscriberObjectDetection()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('distance_calculation node terminated')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.signal_shutdown('ROS stopped')    
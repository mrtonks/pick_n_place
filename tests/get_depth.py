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

class depth_processing():

    def __init__(self):
        rospy.init_node('listener_for_depth', anonymous=True)
        self.bridge = CvBridge()
        self.objects_detected = {}
        
        ### zeroMQ works
        port = "5556"
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt_string(zmq.SUBSCRIBE, u'')
        print('Collecting updates...')
        socket.connect("tcp://127.0.0.1:%s" % port)
        print('waiting...')
        stream = zmqstream.ZMQStream(socket)
        stream.on_recv(self.callback)
        try:
           ioloop.IOLoop.instance().start()
        except KeyboardInterrupt:
           print('\nListener has stopped')

    def callback(self, data):
        print('receiving...')               
        try:            
            self.objects_detected = pickle.loads(data[0])            
            rospy.Subscriber('/zed/zed_node/depth/depth_registered', Image, self.callback_2)
            #sub.unregister()
        except Exception as e:
            print(e)
        
                            
    def callback_2(self, depth_data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
        except CvBridgeError as e:
            print(e)

        depth_array = np.array(depth_image, dtype=np.float32)

        print('Image size: {width}x{height}'.format(width=depth_data.width,height=depth_data.height))

        # u = depth_data.height/2
        # v = depth_data.width/2

        # print('Center depth: {dist} m'.format(dist=depth_array[u,v]))
                
        try:
            count_obj_detected = len(self.objects_detected)
            print 'Number of objects detected: ', count_obj_detected
            obj_distances = np.empty((count_obj_detected))
            obj_names = np.empty((count_obj_detected), dtype="S20")
            print obj_distances.shape
            print obj_names.shape
            for obj_idx in range(count_obj_detected):
                obj_detected = self.objects_detected[str(obj_idx)]
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
            print 'Objects detected', ', '.join(obj_names)          
            print 'Closest object: ', obj_names[closest_obj], ' - ', obj_distances[closest_obj], ' m'            
        except Exception as e:
            print(e)                        
        
        rospy.signal_shutdown('stopping')

if __name__ == "__main__":        
    try:
        detector = depth_processing()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Detector node terminated.")
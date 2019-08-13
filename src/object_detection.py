#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pick and Place
The object detection processes.

Copyright (c) 2019 Jesus Vera
Licensed under the MIT License (see LICENSE for details)
Written by Jesus Vera
"""

import os
import sys
import time

import zmq
import pickle
import numpy as np
from PIL import Image as PILImg

# ROS imports
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# MaskRCNN imports
ROOT_DIR = os.path.abspath('../mrcnn/')  # Root directory of the MaskRCNN libraries
sys.path.append(ROOT_DIR)  # To find local version of the library
import mrcnn.model as modellib
from mrcnn import utils, visualize
from mrcnn.config import Config

from helpers import const, find_contour_angle

# Constant
MODEL_DIR = os.path.abspath("../model/mask_rcnn_cocosynth_dataset_0300.h5")  # Path to model weights

# Global variables
is_moving = False
model = None
start_time = None

class InferenceConfig(Config):
    """
    Configuration class for the inference.
    
    Parameters
    ----------
        Config: ```Config``` from ```mrcnn.config``` 
    """
    NAME = 'inference'
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

    IMAGE_MIN_DIM = 512
    IMAGE_MAX_DIM = 512
    DETECTION_MIN_CONFIDENCE = 0.95  # Increse if detection is returning objects
    NUM_CLASSES = 11
    BACKBONE = 'resnet50'

    RPN_ANCHOR_SCALES = (8, 16, 32, 64, 128)
    TRAIN_ROIS_PER_IMAGE = 32
    MAX_GT_INSTANCES = 50
    POST_NMS_ROIS_INFERENCE = 500
    POST_NMS_ROIS_TRAINING = 1000

def check_moving(data):
    """
    Checks if object is being moved.

    Parameters
    ----------
        data (boolean): Data comes from Rospy.Subscriber.
    """
    global is_moving
    is_moving = data.data

def sendImageCalculationData(objects_detected):
    """
    Use ZMQ to send a message to a python 2 script for obtaining 
    the distances from the objects detected.

    Parameters
    ----------
        objects_detected (json): Object containing the class and coordinates of the bounding box.
    """
    global model
    global is_moving

    print("Sending data for distance calculation...")
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    addr = "127.0.0.1"  # Remote ip or localhost
    port = "5556"  # Same as in the pupil remote gui
    socket.bind('tcp://{}:{}'.format(addr, port))
    time.sleep(1.0)  # Making sure the socket is up

    try:
        socket.send_multipart([pickle.dumps(objects_detected, protocol=2)])  # Python 2 accepts only procotol 2.
        is_moving = True
        print("Data sent!\n")
    except pickle.PicklingError as e:
        print("Error: {}".format(e))
        sys.exit(1)

def getObjectsDetected():
    """Detects objects in the image.
    
    Implementation of the MaskRCNN is done here. 
    """
    global start_time
    global model
    
    while is_moving:
        pass

    # Subsribes only once and stops
    data = rospy.wait_for_message('/zed/zed_node/right_raw/image_raw_color', Image)
    while data is None:
        return

    print("\nStarting object detection...")    
    # Image is BGRA8
    image = PILImg.frombytes(mode='RGBA', size=(data.width, data.height), data=data.data, decoder_name='raw')
    img_array = np.array(image)
    rgb_image = img_array[:, :, [2, 1, 0]] # Convert to RGBA and remove alpha channel

    inference_config = InferenceConfig()
    # Modify max dimension according to the image obtained
    inference_config.IMAGE_MAX_DIM = data.width if data.width > data.height else data.height
    # Initialise model and load weights if it is not initialised yet
    if model is None:
        # Model is initialized in "inference" mode for object detection
        model = modellib.MaskRCNN(mode="inference",
                                config=inference_config,
                                model_dir=os.path.join(ROOT_DIR, 'logs'))
        model.load_weights(MODEL_DIR, by_name=True)
    
    # Get predictions
    try:
        results = model.detect([rgb_image], verbose=1)
    except Exception as e:
        print('Error: {}'.format(e))
        model = None
        sys.exit(1)
    
    r = results[0]
    # Uncomment for visualisation of images with masks and calibration
    visualize.display_instances(rgb_image, r['rois'], r['masks'], r['class_ids'], 
                              const.CLASSES, r['scores'], figsize=(10,10))
    count_classes = len(r['class_ids']) # Count classes
    if count_classes == 0:
        print("No objects found. Decrease min confidence if there is an object.")
        return
    print('Number of objects found: {}'.format(count_classes))
    # Create an object with a dictionary of the objects detected
    objects_detected = {}
    for idx in range(count_classes):
        masks = r['masks']
        obj_info = dict()
        obj_info['name'] = const.CLASSES[r['class_ids'][idx]]  # Obtain class names from ids
        obj_info['coordinates'] = [value for value in r['rois'][idx]]
        obj_info['orientation'] = find_contour_angle.getContourAngle(masks[:, :, idx], 'rad')  # Must be radians
        objects_detected[str(idx)] = obj_info
    sendImageCalculationData(objects_detected)

if __name__ == "__main__":
    rospy.init_node('object_detection', log_level=rospy.INFO)
    print("Taking photo for object detection...")

    # Before running this file, run in a terminal "roslaunch zed_wrapper zed.launch"
    try:
        rospy.Subscriber('is_moving', Bool, check_moving, queue_size=10)
        while True:
            try:                            
                print("Waiting...")
                getObjectsDetected()            
            except KeyboardInterrupt:
                model = None
                print("Done")                
                sys.exit(1)
                break
        rospy.spin()
    except rospy.ROSInterruptException:
        model = None
        rospy.loginfo('object_detection node terminated')    
    
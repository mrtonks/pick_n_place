#!/usr/bin/env python3

import os
import sys
import time
import rospy
import zmq
import pickle
import numpy as np
from PIL import Image as PILImg
from sensor_msgs.msg import Image

ROOT_DIR = os.path.abspath("../Mask_RCNN/") # Root directory of the project
sys.path.append(ROOT_DIR) # To find local version of the library

from mrcnn import utils
import mrcnn.model as modellib
from mrcnn import visualize
from mrcnn.config import Config

MODEL_DIR = os.path.join(ROOT_DIR, "logs/cocosynth_dataset20190724T0156/mask_rcnn_cocosynth_dataset_0300.h5") # Path to weights
# Classes as contained in the coco definitions file
CLASSES = [ 
    'BG', 'cat_cup', 'black_trainer', 'small_tupper',
    'katana_umbrella', 'harrogate_water', 'feet_spray',
    'highland_water', 'catbus', 'snapback_hat', 'unstable_unicorns'
]

start_time = None
model = None

class InferenceConfig(Config):
    NAME = 'inference'
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

    IMAGE_MIN_DIM = 512
    IMAGE_MAX_DIM = 512
    DETECTION_MIN_CONFIDENCE = 0.99
    NUM_CLASSES = 11
    BACKBONE = 'resnet50'
    
    RPN_ANCHOR_SCALES = (8, 16, 32, 64, 128)
    TRAIN_ROIS_PER_IMAGE = 32
    MAX_GT_INSTANCES = 50 
    POST_NMS_ROIS_INFERENCE = 500 
    POST_NMS_ROIS_TRAINING = 1000 

def sendImageCalculationData(objects_detected):
    """ Use pyZMQ to send a message to a python 2 script for
    obtaining the distances from the objects detected.  """
    global model
    print('Sending data for distance calculation...')
    
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    addr = '127.0.0.1'  # remote ip or localhost
    port = '5556'  # same as in the pupil remote gui
    socket.bind('tcp://{}:{}'.format(addr, port))
    # Making sure the socket is up
    time.sleep(1.0)

    try:
        socket.send_multipart([pickle.dumps(objects_detected, protocol=2)])
        print('Data sent!\n\n')
    except pickle.PicklingError as e:
        print('Error: {}'.format(e))
        sys.exit(1)

def getObjectsDetected(data):
    global start_time
    global model

    if start_time == None:
        start_time = time.time()
    elif time.time() - start_time > 15:
        start_time = time.time()
    else:
        return    

    print('Starting object detection...')

    # Image is BGRA8
    image = PILImg.frombytes(mode='RGBA', size=(data.width, data.height), data=data.data, decoder_name='raw')
    # Convert to numpy array
    img_array = np.array(image)
    # Convert to RGBA and remove alpha channel
    rgb_image = img_array[:, :, [2, 1, 0]]

    inference_config = InferenceConfig()
    # Modify max dimension from the image obtained
    inference_config.IMAGE_MAX_DIM = data.width if data.width > data.height else data.height
    # Initialise model and load weights if it is not initialised yet
    if model is None:
        model = modellib.MaskRCNN(mode="inference",
                                config=inference_config,
                                model_dir=os.path.join(ROOT_DIR, 'logs'))
        model.load_weights(MODEL_DIR, by_name=True)
    
    # Get predictions
    try:
        results = model.detect([rgb_image], verbose=1)
    except Exception as e:
        print('Error: ', e)
        sys.exit(1)
    
    r = results[0]
    visualize.display_instances(rgb_image, r['rois'], r['masks'], r['class_ids'], 
                               CLASSES, r['scores'], figsize=(10,10))
    count_classes = len(r['class_ids']) # Count classes
    if count_classes == 0:
        print('No objects found. Decrease min confidence if there is an object.')
        return
    
    print('Objects found: {}'.format(count_classes))
    # Create an object with a dictionary of the objects detected
    objects_detected = {} # Change back to {} if doesn't work
    for idx in range(count_classes):
        obj_info = dict()
        print(CLASSES)
        print(r['class_ids'])
        print(r['class_ids'][idx])
        obj_info['name'] = CLASSES[r['class_ids'][idx]] 
        obj_info['coordinates'] = [value for value in r['rois'][idx]]
        objects_detected[str(idx)] = obj_info

    sendImageCalculationData(objects_detected)

def main():
    rospy.init_node('object_detection', log_level=rospy.INFO)
    print('Taking photo for object detection...')

    rospy.Subscriber('/zed/zed_node/rgb_raw/image_raw_color', Image, getObjectsDetected)
    #rate = rospy.Rate(50)
    try:                
        #rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown('ROS stopped')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.signal_shutdown('ROS stopped')        
    
#!/usr/bin/env python

# import zmq
import sys
import rospy
import pickle
import numpy as np
# from zmq.eventloop import ioloop, zmqstream
from PIL import Image as PILImage
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
#import cv2

import os
import random
import math
import matplotlib
import matplotlib.pyplot as plt

# Root directory of the project
ROOT_DIR = os.path.abspath("../Mask_RCNN/")
sys.path.append(ROOT_DIR)  # To find local version of the library

from mrcnn import utils
import mrcnn.model as modellib
from mrcnn import visualize
from mrcnn.config import Config

import skimage

CLASSES = [ 
    'BG', 'cat_cup', 'black_trainer', 'small_tupper',
    'katana_umbrella', 'harrogate_water', 'feet_spray',
    'highland_water', 'catbus', 'snapback_hat'
]

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

def callback(msg):
    print('received:')
    # img_data = msg.data;
    # img_h = msg.height;
    # img_w = msg.width;
    
    # # Create blank image output with required size
    # img = np.zeros(msg.width,msg.height,3);
    
    # img_r = reshape(imgData(1:3:end),img_w,img_h)';
    # img_g = reshape(imgData(2:3:end),img_w,img_h)';
    # img_b = reshape(imgData(3:3:end),img_w,img_h)';
    # img(:,:,1) = img_r;   
    # img(:,:,2) = img_g;
    # img(:,:,3) = img_b;

    print(msg.header)
    print(msg.encoding)
    print(msg.width, msg.height)
    #bridge = CvBridge()
    #cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    #custom_image = cv2.cvtColor(bridge_image, cv2.COLOR_BGR2RGB)
    image = PILImage.frombytes(mode='RGBA', size=(msg.width, msg.height), data=msg.data, decoder_name="raw")
    img_array = np.array(image)
    rearranged = img_array[:,:,[2,1,0]]
    #img = PILImage.fromarray(rearranged)
    #img.show()
    #image.show()
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    MODEL_DIR = os.path.join(ROOT_DIR, "logs/cocosynth_dataset20190724T0156/mask_rcnn_cocosynth_dataset_0300.h5")
    #print(MODEL_DIR)
    inference_config = InferenceConfig()
    inference_config.IMAGE_MAX_DIM = msg.width if msg.width > msg.height else msg.height
    inference_config.display()
    model = modellib.MaskRCNN(mode="inference",
                              config=inference_config,
                              model_dir=os.path.join(ROOT_DIR, "logs"))
 
    model.load_weights(MODEL_DIR, by_name=True)
    
    results = model.detect([rearranged], verbose=1)
    r = results[0]
    #for i, class_id in zip(range(len(r['class_ids'])), r['class_ids']):
    #    print(CLASSES[class_id], r['scores'][i])
    visualize.display_instances(rearranged, r['rois'], r['masks'], r['class_ids'], 
                                CLASSES, r['scores'], figsize=(8,8))
    #visualize.display_top_masks(rearranged, r['masks'], r['class_ids'], 
    #                            CLASSES, limit=4)
    
    
    ###test_images_dir = '../images'
    ###image_paths = []
    ### for filename in os.listdir(test_images_dir):
    ###     if os.path.splitext(filename)[1].lower() in ['.png', '.jpg', '.jpeg']:
    ##         image_paths.append(os.path.join(test_images_dir, filename))


    # for image_path in image_paths:
    #     img = skimage.io.imread(image_path)
    #     img_arr = np.array(img)
    #     results = model.detect([img_arr], verbose=1)
    #     #results = model.detect([rearranged], verbose=1)
    #     r = results[0]
    #     print(image_path)
    #     for class_id in r['class_ids']:
    #         print(CLASSES[class_id - 1])
        #print(r)
    rospy.signal_shutdown('stopping')



#{"supercategory": "cup", "id": 1, "name": "cat_cup"}, 
# {"supercategory": "shoe", "id": 2, "name": "black_trainer"}, 
# {"supercategory": "tupper", "id": 3, "name": "small_tupper"}, 
# {"supercategory": "umbrella", "id": 4, "name": "katana_umbrella"}, 
# {"supercategory": "bottle", "id": 5, "name": "harrogate_water"}, 
# {"supercategory": "bottle", "id": 6, "name": "feet_spray"}, 
# {"supercategory": "bottle", "id": 7, "name": "highland_water"}, 
# {"supercategory": "plush", "id": 8, "name": "catbus"}, 
# {"supercategory": "hat", "id": 9, "name": "snapback_hat"}, 
# {"supercategory": "cardgame", "id": 10, "name": "unstable_unicorns"}
#{"supercategory": "bottle", "id": 1, "name": "highland_water"}, {"supercategory": "bottle", "id": 2, "name": "harrogate_water"}, {"supercategory": "bottle", "id": 3, "name": "feet_spray"}, {"supercategory": "umbrella", "id": 4, "name": "katana_umbrella"}, {"supercategory": "cardgame", "id": 5, "name": "unstable_unicorns"}, {"supercategory": "hat", "id": 6, "name": "snapback_hat"}, {"supercategory": "shoe", "id": 7, "name": "black_trainer"}, {"supercategory": "cup", "id": 8, "name": "cat_cup"}, {"supercategory": "tupper", "id": 9, "name": "small_tupper"}, {"supercategory": "plushy", "id": 10, "name": "catbus"}

def process_message(msg):
    print('received:', msg)
    # obj = pickle.loads(msg[0])
    # print(obj.names, obj.x, obj.y, obj.theta)

def listener():
    ### ROSPY works on python 3
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/zed/zed_node/rgb_raw/image_raw_color', Image, callback)
    # #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
   
    ### zeroMQ works
    # port = "5556"
    # context = zmq.Context()
    # socket = context.socket(zmq.SUB)
    # socket.setsockopt_string(zmq.SUBSCRIBE, '')
    # print("Collecting updates...")
    # socket.connect("tcp://127.0.0.1:%s" % port)
    # print('waiting...')
    # stream = zmqstream.ZMQStream(socket)
    # stream.on_recv(process_message)
    # try:
    #    ioloop.IOLoop.instance().start()
    # except KeyboardInterrupt:
    #    print("\nListener has stopped")

if __name__ == '__main__':
    # Local path to trained weights file
    listener()

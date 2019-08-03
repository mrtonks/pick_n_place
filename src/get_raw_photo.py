#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

def handleGetPhoto():
    image_msg = rospy.wait_for_message('/zed/zed_node/right_raw/image_raw_color', Image)
    return image_msg  
    

def getRawPhotoServer():
    rospy.init_node('get_photo_server')
    srv = rospy.Service('get_raw_photo', Image, handleGetPhoto)
    print('Ready to get photos.')
    rospy.spin()

if __name__ == "__main__":
    getRawPhotoServer()
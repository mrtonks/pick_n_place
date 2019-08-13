#!/usr/bin/env python

import rospy
import numpy as np
from PIL import Image as PILImage
from sensor_msgs.msg import Image

def callback(msg):    
    image = PILImage.frombytes(mode='RGBA', size=(msg.width, msg.height), data=msg.data, decoder_name="raw")
    img_array = np.array(image)
    rearranged = img_array[:,:,[2,1,0]]
    img = PILImage.fromarray(rearranged)
    img.show()
    rospy.signal_shutdown('bye')

def listener():
    rospy.init_node('listener_for_depth', anonymous=True)
    rospy.Subscriber('/zed/zed_node/rgb_raw/image_raw_color', Image, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
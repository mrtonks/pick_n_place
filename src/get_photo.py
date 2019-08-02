import rospy
from sensor_msgs.msg import Image

def sendPhotoToCaller(data):
    return data
    

def getPhotoFromCamera():
    rospy.Subscriber('/zed/zed_node/right_raw/image_raw_color', Image, sendPhotoToCaller) 
    return data   
    

def getPhotoServer():
    rospy.init_node('get_photo_server')
    srv = rospy.Service('get_photo', Image, getPhotoFromCamera)
    print 'Ready to get photos.'
    rospy.spin()

if __name__ == "__main__":
    
#!/usr/bin/env python2

import sys
import time
import zmq
import pickle
from pick_n_place.msg import ObjectInfo
# import rospy
#from std_msgs.msg import String

def talker():
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    # rospy.init_node('talker', anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()
    object_info = ObjectInfo()
    object_info.names = ['', '', '']
    object_info.x = [0, 0, 0]
    object_info.y = [0, 0, 0]
    object_info.theta = [0, 0, 0]
    # object_info = {
    #     "names": ['', '', ''],
    #     "x": [0, 0, 0],
    #     "y": [0, 0, 0],
    #     "theta": [0, 0, 0]
    # }
    

    print 'talker'
    ctx = zmq.Context()
    s = ctx.socket(zmq.PUB)
    s.bind("tcp://*:5556")
    time.sleep(1.0)

    while True:
        try:
            #hello_str = "hello world %s"  % time.clock()
            s.send_multipart([pickle.dumps(object_info)])
            print pickle.dumps(object_info)
            time.sleep(0.1)
            print "Done------------------------------"
        except KeyboardInterrupt:
            sys.exit()

if __name__ == '__main__':
    try:
        talker()
    except zmq.ZMQError:
        pass

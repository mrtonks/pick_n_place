#!/usr/bin/env python

import zmq
# import rospy
import pickle
from zmq.eventloop import ioloop, zmqstream
from std_msgs.msg import String
from turtlesim.msg import Pose

def callback(data):
    print('received:', data)
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def process_message(msg):
    print('received:', msg)
    # obj = pickle.loads(msg[0])
    # print(obj.names, obj.x, obj.y, obj.theta)

def listener():
    ### ROSPY works on python 3
    # rospy.init_node('listener', anonymous=True)
    # rospy.Subscriber('/turtle1/pose', Pose, callback)
    # #spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
   
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
    listener()

#!/usr/bin/env python

import sys
import struct
import copy

# rospy - ROS Python API
import rospy
import baxter_interface

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

from std_msgs.msg import Header

from sensor_msgs.msg import JointState

from geometry_msgs.msg import (
    PoseStamped,
    Pose, 
    Point,
    Quaternion
)

# From ik_pick_and_place_demo.py
# First run: rosrun baxter_interface joint_trajectory_action_server.py

class PickAndPlace(object):
    def __init__(self, limb, hover_distance, verbose =True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
    
    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_join_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)
    
    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)
    
    def _approach(self, pose):
        """ Positions grip hovering above object """
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        """ Positions grip on object """
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    # rostopic echo -n 1 /robot/joint_states
    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()
    
    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def initplannode(goal, limb):
    rospy.init_node("planning_node")
    # Limb = 'right'
    hover_distance = 0.15 # meters
    # Start position
    starting_joint_angles = {'right_w0': 3.0504135967740122,
                             'right_w1': -0.8109176937839067,
                             'right_w2': 2.6935388551620534,
                             'right_e0': -0.012537762734409874,
                             'right_e1': 2.0758228731051833,
                             'right_s0': 0.28086350234632906,
                             'right_s1': -1.3852461282141109}
    pnp =  PickAndPlace(limb, hover_distance)
    overhead_orientation = Quaternion(
        x = 0.0147500446928,
        y = 0.998939783418,
        z = -0.0395300159478,
        w = 0.0184152959542
    )

    object_pose = (Pose(
        position=Point(x = goal['x'], y = goal['y'], z = goal['z']),
        orientation=overhead_orientation))
    # block_poses.append(Pose(
    #     position=Point(x = 0.38459808171, y = -0.542247604291, z = -0.20678082182),
    #     orientation=overhead_orientation)
    # )
    # block_poses.append(Pose(
    #     position=Point(x = 1.005513915, y = -0.604036611297, z = -0.18191294902),
    #     orientation=overhead_orientation)
    # )
    # Move to the desired starting angles
    pnp.move_to_start(starting_joint_angles)  
    print 'Picking...'
    pnp.pick(object_pose)
    return
#!/usr/bin/env python

import sys
import struct
import copy
import random
import numpy as np

# ROS imports
import rospy
import baxter_interface
from tf.transformations import quaternion_from_euler
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

from const import (
    HOVER_DISTANCE,
    LIMB,
    OVERHEAD_ORIENTATION,
    START_JOINT_ANGLES,
    Y_PLACING
)

is_obj_picked = False

# Taken from Rethink Robotics ik_pick_and_place_demo.py
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
        print "Getting robot state... "
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print "Enabling robot... "
        self._rs.enable()
    
    def move_to_start(self, start_angles=None):
        print "Moving the {0} arm to start pose...".format(self._limb_name)
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print "Running. Ctrl-c to quit"

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
                print "IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print "IK Joint Solution:\n{0}".format(limb_joints)
                print "------------------"
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

    # rost05opic echo -n 1 /robot/joint_states
    def pick(self, pose):
        global is_obj_picked
        self._gripper.calibrate()
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object        
        if self._gripper.force() > 25:
            print "Gripper force = {}".format(self._gripper.force())
            is_obj_picked = True
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

def initplannode(goal, quat, limb): 
    global is_obj_picked
    is_obj_picked = False
    
    pnp =  PickAndPlace(limb, HOVER_DISTANCE, False)
    object_poses = []
    quat_object = Quaternion(
        x=quat[0],
        y=quat[1],
        z=quat[2],
        w=quat[3]
    )
    # Object pose
    object_poses.append(Pose(
        position=Point(x = goal[0], y = goal[1], z = goal[2]),
        orientation=quat_object))
    
    # Place the object to left or right, 
    # depending whether the object is to the left or right from the center
    if goal[1] < 0:
        goal[1] = goal[1] + Y_PLACING
    else:
        goal[1] = goal[1] - Y_PLACING
    # Generate placing quaternions orientation
    z_angle_placing = -random.randint(1, 179)
    quat_placing = quaternion_from_euler(np.deg2rad(176.0), 0.0, np.deg2rad(z_angle_placing))
    if quat_placing[1] < 0:
        # The y quaternion comes negative and it must be positive 
        # or the gripper will not use the correct angle value
        quat_placing[1] = -quat_placing[1]
    # Placing pose
    object_poses.append(Pose(
        position=Point(x = goal[0], y = goal[1], z = goal[2]),
        orientation=Quaternion(x=quat_placing[0], y=quat_placing[1], z=quat_placing[2], w=quat_placing[3])))

    try:
        # Move to the desired starting angles
        pnp.move_to_start(START_JOINT_ANGLES)  
        print "Picking..."
        pnp.pick(object_poses[0])
        print "Placing..."
        pnp.place(object_poses[1])
        print "Returning..."
        pnp.move_to_start(START_JOINT_ANGLES)
    except (Exception, KeyboardInterrupt) as e:
        rospy.logerr('Error: {}'.format(e))
    return is_obj_picked
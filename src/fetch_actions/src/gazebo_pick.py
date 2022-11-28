#! /usr/bin/python3

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import copy
import actionlib
import rospy
import numpy as np
from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.srv import GetPlan, GetPlanResponse
from fetch_actions.msg import MoveBaseRequestAction
from fetch_actions.msg import TorsoControlRequestAction
from fetch_actions.msg import PointHeadRequestAction
from fetch_actions.msg import PickRequestAction
from tf.transformations import euler_from_quaternion
import tf



# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()
        self.server = actionlib.SimpleActionServer("kisailus_move_base", MoveBaseRequestAction, self.request_callback, auto_start=False)
        self.server.start()
        self.make_plan_client = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        self.cur_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_cb)
        self.cur_pose = PoseStamped()
        self.pose_received = False
    
    def pose_cb(self, pose_msg):
        self.pose_received = True
        self.cur_pose.header = pose_msg.header
        self.cur_pose.pose = pose_msg.pose.pose
        # self.cur_pose = pose_msg.pose.pose
    
    def request_callback(self, msg):
        rospy.loginfo("MOVE BASE: Received request...")
        suc, x, y, theta = self.plan_to(msg.x, msg.y, msg.theta)
        if suc:
            self.goto(x, y, theta)
        else:
            self.server.set_aborted()
            return
        rospy.loginfo("MOVE BASE: Action completed")
        self.server.set_succeeded()
    
    def plan_to(self, x, y, theta):
        if not self.pose_received:
            rospy.logwarn("[MOVE_BASE]: No pose received yet. Cannot plan path")
            return False, 0, 0, 0
        start = self.cur_pose
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = sin((theta)/2.0) # add t to make sure robot is still facing the goal location
        goal.pose.orientation.w = cos((theta)/2.0)
        resp = self.make_plan_client(start, goal, 0.001)
        if len(resp.plan.poses) > 0:
            # plan directly to goal
            quaternion = (
                goal.pose.orientation.x,
                goal.pose.orientation.y,
                goal.pose.orientation.z,
                goal.pose.orientation.w)
            euler = euler_from_quaternion(quaternion)
            yaw = euler[2]
            return True, goal.pose.position.x, goal.pose.position.y, yaw
        else:
            # attempt to plan near goal             
            thetas = np.linspace(0, 2*np.pi, num=20)
            offsets = [0.6, 0.7, 0.8, 0.9, 1.0] #[0.5, 0.6, 0.7, 0.8] #[i*0.25 for i in range(2,4)]
            for radius in offsets:
                for t in thetas:
                    goal = PoseStamped()
                    goal.header.stamp = rospy.Time.now()
                    goal.header.frame_id = 'map'
                    goal.pose.position.x = x + radius*np.cos(t)
                    goal.pose.position.y = y + radius*np.sin(t)
                    goal.pose.orientation.z = sin((t+np.pi)/2.0) # add t to make sure robot is still facing the goal location
                    goal.pose.orientation.w = cos((t+np.pi)/2.0)
                    resp = self.make_plan_client(start, goal, 0.001)
                    if len(resp.plan.poses) > 0:
                        quaternion = (
                            goal.pose.orientation.x,
                            goal.pose.orientation.y,
                            goal.pose.orientation.z,
                            goal.pose.orientation.w)
                        euler = euler_from_quaternion(quaternion)
                        yaw = euler[2]
                        return True, goal.pose.position.x, goal.pose.position.y, yaw
        rospy.logwarn("No valid path found!")
        return False, -1, -1, -1


    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.server = actionlib.SimpleActionServer("kisailus_torso_controller", TorsoControlRequestAction, self.callback, auto_start=False)
        self.server.start()
        self.joint_names = joint_names
    
    def callback(self, msg):
        rospy.loginfo("TORSO CONTROLLER: Received request...")
        self.move_to([msg.height])
        rospy.loginfo("TORSO CONTROLLER: Action completed")
        self.server.set_succeeded()

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()
        self.server = actionlib.SimpleActionServer("kisailus_point_head", PointHeadRequestAction, self.callback, auto_start=False)
        self.server.start()
    
    def callback(self, msg):
        rospy.loginfo("POINT HEAD: Received request")
        self.look_at(msg.x, msg.y, msg.z)
        rospy.loginfo("POINT HEAD: Action completed")
        self.server.set_succeeded()

    def look_at(self, x, y, z, frame='map', duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

# Tools for grasping
class GraspingClient(object):

    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=False)
        self.move_group = MoveGroupInterface("arm", "base_link")

        find_topic = "basic_grasping_perception/find_objects"
        rospy.loginfo("Waiting for %s..." % find_topic)
        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
        self.find_client.wait_for_server()

        self.request_server = actionlib.SimpleActionServer('kisailus_pick', PickRequestAction, self.callback, auto_start=False)
        self.request_server.start()
        self.grasped_obj = None
        self.surfaces = []
        self.tf_listener = tf.TransformListener()
    
    def callback(self, request):
        rospy.loginfo("PICK_NODE: Got a request..")
        if request.mode == 0:
            self.updateScene()
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose = request.pick_pose
            self.tf_listener.waitForTransform("base_link", "map", rospy.Time.now(), rospy.Duration(90))
            bl_pose_goal = self.tf_listener.transformPose("base_link", p)
            rospy.logwarn("Particle in base_link: ({}, {}, {})".format(bl_pose_goal.pose.position.x, bl_pose_goal.pose.position.y, bl_pose_goal.pose.position.z))
            cube, grasps = self.getGraspableCube(bl_pose_goal.pose)
            if cube == None:
                rospy.logwarn("Perception failed.")
                self.request_server.set_aborted()
            # Pick the object
            if self.pick(cube, grasps):
                self.grasped_obj = cube
                self.request_server.set_succeeded()
            else:
                rospy.logwarn("Grasping failed.")
                self.request_server.set_aborted()
        elif request.mode == 1:
            self.tuck()
        elif request.mode == 2:
            self.updateScene()
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose = request.pick_pose
            # rospy.logwarn("Surface z pos: {}".format(self.surfaces[0].primitive_poses[0].position.z))
            # rospy.logwarn("Surface z dim: {}".format(self.surfaces[0].primitives[0].dimensions[2]))
            # rospy.logwarn("Grasp obj z dim: {}".format(self.grasped_obj.primitives[0].dimensions[2]))
            p.pose.position.z = self.surfaces[0].primitives[0].dimensions[2] + self.grasped_obj.primitives[0].dimensions[2]/2# + 0.03 #self.surfaces[0].primitive_poses[0].position.z + self.surfaces[0].primitives[0].dimensions[2]/2 + self.grasped_obj.primitives[0].dimensions[2]/2
            p.pose.orientation.w = 1.0
            rospy.logwarn("Place location is ({}, {}, {:.4f})".format(p.pose.position.x, p.pose.position.y, p.pose.position.z))
            if self.place(self.grasped_obj, p):
                self.request_server.set_succeeded()
                self.grasped_obj = None
            else:
                self.request_server.set_aborted()


    def updateScene(self):
        # find objects
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = True
        self.find_client.send_goal(goal)
        rospy.logwarn("[GRASPING CLIENT] Waiting for find result")
        self.find_client.wait_for_result()
        rospy.logwarn("[GRASPING CLIENT] Got result")
        find_result = self.find_client.get_result()

        # remove previous objects
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, False)
        for name in self.scene.getKnownAttachedObjects():
            self.scene.removeAttachedObject(name, False)
        self.scene.waitForSync()

        # insert objects to scene
        idx = -1
        # rospy.logwarn("[BASIC GRASPING]: Found {} objects".format(len(find_result.objects)))
        for obj in find_result.objects:
            height = obj.object.primitive_poses[0].position.z
            if height < 0.3: # ignore low objects
                continue
            if np.sqrt(obj.object.primitive_poses[0].position.x**2 + obj.object.primitive_poses[0].position.y**2) > 1.5: # ignore objects far away from bot
                continue
            idx += 1
            obj.object.name = "object%d"%idx
            self.scene.addSolidPrimitive(obj.object.name,
                                         obj.object.primitives[0],
                                         obj.object.primitive_poses[0])
        for obj in find_result.support_surfaces:
            height = obj.primitive_poses[0].position.z
            if height < 0.3: # ignore low objects
                continue
            # if np.sqrt(obj.primitive_poses[0].position.x**2 + obj.primitive_poses[0].position.y**2) > 1.5: # ignore objects far away from bot
            #     continue
            # extend surface to floor, and make wider since we have narrow field of view
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
                                            1.5,  # wider
                                            obj.primitives[0].dimensions[2] + height]
            obj.primitive_poses[0].position.z += -height/2.0
            # rospy.logwarn(obj)
            # rospy.logwarn("------------------------")

            # add to scene
            self.scene.addSolidPrimitive(obj.name,
                                         obj.primitives[0],
                                         obj.primitive_poses[0])
            self.surfaces.append(obj)

        self.scene.waitForSync()

        # store for grasping
        self.objects = find_result.objects
        self.surfaces = find_result.support_surfaces

    def getGraspableCube(self, goal):
        graspable = None
        for obj in self.objects:
            obj_x = obj.object.primitive_poses[0].position.x
            obj_y = obj.object.primitive_poses[0].position.y
            obj_z = obj.object.primitive_poses[0].position.z
            dist_to_goal = np.sqrt((obj_x-goal.position.x)**2 + (obj_y-goal.position.y)**2 + (obj_z-goal.position.z)**2)
            rospy.logwarn("Dist from {} to goal is {}".format(obj.object.name, dist_to_goal))
            if dist_to_goal > 0.3:
                continue
            # need grasps
            if len(obj.grasps) < 1:
                continue
            # check distance
            if obj.object.primitive_poses[0].position.x >= 0.8:
                continue
            # if obj.object.primitives[0].dimensions[0] < 0.05 or \
            #    obj.object.primitives[0].dimensions[0] > 0.07 or \
            #    obj.object.primitives[0].dimensions[0] < 0.05 or \
            #    obj.object.primitives[0].dimensions[0] > 0.07 or \
            #    obj.object.primitives[0].dimensions[0] < 0.05 or \
            #    obj.object.primitives[0].dimensions[0] > 0.07:
            #     continue
            # has to be on table
            if obj.object.primitive_poses[0].position.z < 0.5:
                continue
            rospy.logwarn("Found a graspable object with name: {} at loc ({}, {}, {})".format(obj.object.name, obj.object.primitive_poses[0].position.x, obj.object.primitive_poses[0].position.y, obj.object.primitive_poses[0].position.z))
            return obj.object, obj.grasps
        # nothing detected
        return None, None

    def getSupportSurface(self, name):
        for surface in self.support_surfaces:
            if surface.name == name:
                return surface
        return None

    def getPlaceLocation(self):
        pass

    def pick(self, block, grasps):
        # for i,grasp in enumerate(grasps):
        #     grasp_pose = grasp.grasp_pose.pose
        #     rospy.logwarn("Grasp pose {}: ({},{},{})".format(i, grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z))
        # for grasp in grasps:
        #     pg_app = grasp.pre_grasp_approach
        #     frame = pg_app.direction.header.frame_id
        #     x, y, z = pg_app.direction.vector.x, pg_app.direction.vector.y, pg_app.direction.vector.z
        #     # x, y, z, w = grasp.grasp_pose.pose.orientation.x, grasp.grasp_pose.pose.orientation.y, grasp.grasp_pose.pose.orientation.z, grasp.grasp_pose.pose.orientation.w
        #     # (roll, pitch, yaw) = euler_from_quaternion([x, y, z, w])
        #     rospy.logwarn("frame: {}... orientation: ({}, {}, {})".format(frame, x, y, z))
        success, pick_result = self.pickplace.pick_with_retry(block.name,
                                                              grasps,
                                                              support_name=block.support_surface,
                                                              scene=self.scene)
        self.pick_result = pick_result
        return success

    def place(self, block, pose_stamped):
        places = list()
        l = PlaceLocation()
        l.place_pose.pose = pose_stamped.pose
        l.place_pose.header.frame_id = pose_stamped.header.frame_id

        # copy the posture, approach and retreat from the grasp used
        l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
        l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
        places.append(copy.deepcopy(l))
        # create another several places, rotate each by 360/m degrees in yaw direction
        m = 16 # number of possible place poses
        pi = 3.141592653589
        for i in range(0, m-1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)
            places.append(copy.deepcopy(l))

        success, place_result = self.pickplace.place_with_retry(block.name,
                                                                places,
                                                                scene=self.scene)
        return success

    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

if __name__ == "__main__":
    # Create a node
    rospy.init_node("gazebo_pick_node")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    move_base = MoveBaseClient()
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()
    grasping_client = GraspingClient()
    rospy.logwarn("[FETCH ACTIONS]: Loaded all actions")
    while not rospy.is_shutdown():
        rospy.spin()
        
    
    # # Move the base to be in front of the table
    # # Demonstrates the use of the navigation stack
    # rospy.loginfo("Moving to table...")
    # move_base.goto(6.85, -5.1, -1.57079632679)
    # # move_base.goto(2.750, 3.118, 0.0)

    # # # Raise the torso using just a controller
    # rospy.loginfo("Raising torso...")
    # torso_action.move_to([0.4, ])

    # # # Point the head at the cube we want to pick
    # head_action.look_at(7, -6, 1.0, "map")

    # # # Get block to pick
    # while not rospy.is_shutdown():
    #     rospy.loginfo("Picking object...")
    #     grasping_client.updateScene()
    #     cube, grasps = grasping_client.getGraspableCube()
    #     if cube == None:
    #         rospy.logwarn("Perception failed.")
    #         continue

    #     # Pick the block
    #     if grasping_client.pick(cube, grasps):
    #         break
    #     rospy.logwarn("Grasping failed.")

    # # # Tuck the arm
    # grasping_client.tuck()

    # # # Lower torso
    # rospy.loginfo("Lowering torso...")
    # torso_action.move_to([0.0, ])

    # rospy.loginfo("Done!")

    # # Move to second table
    # rospy.loginfo("Moving to second table...")
    # move_base.goto(-3.53, 3.75, 1.57)
    # move_base.goto(-3.53, 4.15, 1.57)

    # # Raise the torso using just a controller
    # rospy.loginfo("Raising torso...")
    # torso_action.move_to([0.4, ])

    # # Place the block
    # while not rospy.is_shutdown():
    #     rospy.loginfo("Placing object...")
    #     pose = PoseStamped()
    #     pose.pose = cube.primitive_poses[0]
    #     pose.pose.position.z += 0.05
    #     pose.header.frame_id = cube.header.frame_id
    #     if grasping_client.place(cube, pose):
    #         break
    #     rospy.logwarn("Placing failed.")

    # # Tuck the arm, lower the torso
    # grasping_client.tuck()
    # torso_action.move_to([0.0, ])

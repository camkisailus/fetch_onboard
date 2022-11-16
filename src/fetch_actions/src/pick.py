#! /usr/bin/python

import rospy
import sys
import actionlib
import tf
import copy

from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown, MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Point, Pose
from moveit_msgs.msg import Grasp
from trajectory_msgs.msg import JointTrajectoryPoint
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from fetch_actions.msg import PickRequestAction
# from grasploc_wrapper_msgs.msg import GrasplocRequestAction, GrasplocRequestGoal
from semantic_frame_mapping.msg import ObjectDetection

class GraspableObject:
    def __init__(self, name, pose):
        self.name = name
        self.pose = pose

class GraspClient:
    def __init__(self):
        rospy.logwarn("Starting init Grasp Client")
        roscpp_initialize(sys.argv)
        rospy.logwarn("rcpp_init done")
        self.tf_listener = tf.TransformListener()
        rospy.logwarn("tf listener created")
        self.scene = PlanningSceneInterface()
        rospy.logwarn("planning scene interface created")
        self.arm = MoveGroupCommander('arm')
        rospy.logwarn("move group commander created")
        self.find_client = actionlib.SimpleActionClient('basic_grasping_perception/find_objects', FindGraspableObjectsAction)
        self.find_client.wait_for_server()
        # rospy.logwarn("Connected to find client")
        # rospy.logwarn("planning scene interface created")
        # self.gloc_client = actionlib.SimpleActionClient('grasploc_requests', GrasplocRequestAction)
        # self.gloc_client.wait_for_server()
        self.request_server = actionlib.SimpleActionServer('kisailus_pick', PickRequestAction, self.callback, auto_start=False)
        self.request_server.start()
        rospy.logwarn("request server started")
        # self.observation_pub = rospy.Publisher("scene/observations", ObjectDetection, queue_size=10)
        rospy.sleep(1)
        rospy.logwarn("Done init pick Grasp Client")
        # self.grasploc_sub = rospy.Subscriber('grasploc', PoseArray, self.grasploc_cb)
    
    def callback(self, request):
        rospy.logwarn("PICK_NODE: Got a request w mode: {}".format(request.mode))
        self.update_scene(request.pick_pose)
        self.send_grasps()
        # obj_to_grasp = 
        # if request.mode == 0:
        #     goal = GrasplocRequestGoal()
        #     goal.tmp = 0
        #     self.gloc_client.send_goal(goal)
        #     self.gloc_client.wait_for_result()
        #     result = self.gloc_client.get_result()
        #     rospy.loginfo("PICK_NODE: Got result from gloc_!")
        # elif request.mode == 1:
        # rospy.logwarn("Updating Scene")
        # cur_pose = self.arm.get_current_pose()
        # rospy.logwarn(cur_pose)
        # self.update_scene()

        # cur_pose = self.arm.get_current_pose()
        # rospy.logwarn(cur_pose)
        # zero_rot = [0,0,0,1]
        # goal_quat = [ -0.7833269, 0, 0, 0.62161 ] # -1.8 rad
        # # goal_quat = [-0.7071068, 0, 0, 0.7071068] # -1.57 rad
        # goal_pos = cur_pose.pose.position
        # rospy.logwarn(request.pick_pose)
        
        # rospy.logwarn(goal_pose)
        
        # rospy.logwarn("Plan succeeded")
        # 
        # 
        # rospy.logwarn(cur_pose)
        # self.scene.add_box("Cooler_wall", surface_pose, (surface.primitives[0].dimensions[0], surface.primitives[0].dimensions[1], surface_pose.pose.position.z*2))
        # if request.mode == 0:
        #     self.go_to_pose_goal(request.pick_pose)
        # elif request.mode == 1:
        #     rospy.logwarn("Planning cartesian move")
        #     cur_pose = self.arm.get_current_pose()
        #     goal_pose = PoseStamped()
        #     goal_pose.header.frame_id = "base_link"
        #     goal_pose.pose = request.pick_pose
        #     # goal_pose.pose.position.x = cur_pose.pose.position.x - 0.04
        #     # goal_pose.pose.position.y = cur_pose.pose.position.y - 0.2
        #     # goal_pose.pose.position.z = cur_pose.pose.position.z 
        #     # goal_pose.pose.orientation.x = cur_pose.pose.orientation.x
        #     # goal_pose.pose.orientation.y = cur_pose.pose.orientation.y
        #     # goal_pose.pose.orientation.z = cur_pose.pose.orientation.z
        #     # goal_pose.pose.orientation.w = cur_pose.pose.orientation.w
        #     waypoints = [cur_pose.pose, goal_pose.pose]
        #     (plan, _) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)
        #     self.arm.execute(plan, wait=True)
        # self.send_grasps(grasp_poses=[request.pick_pose])
            
            # rospy.sleep(3)
            # self.send_grasps(result.graspable_points.poses)
            # rospy.loginfo("PICK_NODE: Done!")
            # # # self.pour()
            # self.place()

        # self.push_elevator(request.pose)        
        self.request_server.set_succeeded()
    
    def pour(self, goal=None):
        cur_pose = self.arm.get_current_pose()
        zero_rot = [0,0,0,1]
        goal_quat = [ -0.7833269, 0, 0, 0.62161 ] # -1.8 rad
        # goal_quat = [-0.7071068, 0, 0, 0.7071068] # -1.57 rad
        goal_pos = cur_pose.pose.position
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "base_link"
        goal_pose.pose.position  = goal_pos
        goal_pose.pose.orientation.x = goal_quat[0]
        goal_pose.pose.orientation.y = goal_quat[1]
        goal_pose.pose.orientation.z = goal_quat[2]
        goal_pose.pose.orientation.w = goal_quat[3]
        waypoints = [cur_pose.pose, goal_pose.pose]
        (plan, _) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)
        rospy.logwarn("Plan succeeded")
        self.arm.execute(plan, wait=True)
    
    def place(self, goal=None):
        cur_pose = self.arm.get_current_pose()
        goal_pose = Pose()
        goal_pose.orientation.x = 0.0
        goal_pose.orientation.y = 0.0
        goal_pose.orientation.z = 0.0
        goal_pose.orientation.w = 1.0
        goal_pose.position.x = 0.848478094621
        goal_pose.position.y = 0.0885518756109
        goal_pose.position.z = 0.84
        waypoints = [cur_pose.pose, goal_pose]
        (plan, _) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.arm.execute(plan, wait=True)
    
    def push_elevator(self, goal):
        # Close to button
        pose_goal = PoseStamped()
        pose_goal.pose = goal
        pose_goal.pose.position.x -= 0.35
        pose_goal.pose.position.z += (0.13)
        # pose_goal.pose.orientation.w = 1.0
        # pose_goal.pose.position.x = goal.x
        # pose_goal.pose.position.y = goal.y
        # pose_goal.pose.position.z = goal.z
        pose_goal.header.frame_id = "map"
        self.tf_listener.waitForTransform("base_link", "map", rospy.Time.now(), rospy.Duration(90))
        bl_pose_goal = self.tf_listener.transformPose("base_link", pose_goal)
        rospy.logwarn(bl_pose_goal)
        
        if self.go_to_pose_goal(bl_pose_goal):
            # Cartesian move to push button
            waypoints = []
            scale = 0.01
            for i in range(19):
                bl_pose_goal.pose.position.x += scale * 1  # Push
                waypoints.append(copy.deepcopy(bl_pose_goal.pose))
            (plan, _) = self.arm.compute_cartesian_path(
                waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
            )  # jump_threshold

            self.arm.execute(plan, wait=True)
            self.request_server.set_succeeded()
            return
        self.request_server.set_aborted()

    
    def go_to_pose_goal(self, pose):
        rospy.logwarn("Setting pose goal")
        self.arm.set_pose_target(pose)
        rospy.logwarn("Sending pose goal to arm")
        success = self.arm.go(wait=True)
        rospy.logwarn("Returned from go")
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success


    def send_grasps(self, grasp_poses=None):
        obj_to_grasp = self.graspable_objs[0]
        grasps = []
        if grasp_poses is None:
            grasp_poses = [self.graspable_objs[0].pose]
        for grasp_pose in grasp_poses:
            grasp = Grasp()
            grasp.id = 'test'
            grasp.grasp_pose.pose = grasp_pose.pose
            grasp.grasp_pose.pose.position.x -=0.15
            grasp.grasp_pose.header.stamp = rospy.Time.now()
            # grasp.grasp_pose.pose = grasp_pose
            # if isinstance(grasp_pose, PoseStamped):
            #     grasp.grasp_pose.pose = grasp_pose
            # elif isinstance(grasp_pose, Pose):
            #     grasp.grasp_pose.pose = grasp_pose
            #     grasp.grasp_pose.header.stamp = rospy.Time.now()
                # grasp.grasp_pose.header.frame_id = 'base_link'
            # print(grasp.grasp_pose)
            # if grasp.grasp_pose.pose.pose.position.z < 0.8:
            #     grasp.grasp_pose.pose.pose.position.z = 0.8
            grasp.grasp_pose.header.frame_id = 'base_link'

            grasp.pre_grasp_posture.joint_names = ['r_gripper_finger_joint', 'l_gripper_finger_joint']
            pos = JointTrajectoryPoint()
            pos.positions.append(0.05) # all the way open
            pos.positions.append(0.05)
            pos.time_from_start = rospy.Duration(3)
            grasp.pre_grasp_posture.points.append(pos)

            # set grasp posture
            grasp.grasp_posture.joint_names = ['r_gripper_finger_joint', 'l_gripper_finger_joint']
            pos = JointTrajectoryPoint()
            pos.positions.append(0.0) # close grippers
            pos.positions.append(0.0)
            pos.effort.append(0.0)
            pos.effort.append(0.0)
            grasp.grasp_posture.points.append(pos)

            grasp.pre_grasp_approach.direction.header.frame_id = 'wrist_roll_link'
            grasp.pre_grasp_approach.direction.vector.x = 1.0
            grasp.pre_grasp_approach.min_distance = 0.10
            grasp.pre_grasp_approach.desired_distance = 0.17

            grasp.post_grasp_retreat.direction.header.frame_id = 'base_link'
            grasp.post_grasp_retreat.direction.vector.z = 1.0
            grasp.post_grasp_retreat.min_distance = 0.10
            grasp.post_grasp_retreat.desired_distance = 0.17

            grasp.allowed_touch_objects = [obj_to_grasp.name]
            grasps.append(grasp)
        
        self.arm.pick(obj_to_grasp.name, grasps, plan_only=True)

    def update_scene(self, pick_pose):
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = True
        self.find_client.send_goal(goal)
        self.find_client.wait_for_result()
        find_result = self.find_client.get_result()
        idx = -1
        # rospy.loginfo("Found {} objects".format(len(find_result.objects)))
        self.graspable_objs = []
        # if len(find_result.objects) == 0:
        #     rospy.logwarn("No graspable objects here")
        #     self.request_server.set_aborted()
        #     return

        for obj in find_result.objects:
            # check distance from objects to our pick pose
            xdist = (obj.object.primitive_poses[0].position.x - pick_pose.position.x)**2
            ydist = (obj.object.primitive_poses[0].position.y - pick_pose.position.y)**2
            zdist = (obj.object.primitive_poses[0].position.z - pick_pose.position.z)**2
            if (xdist+ydist+zdist)**(1/2) > 5.0 or obj.object.primitive_poses[0].position.x >= 1.0:
            # if obj.object.primitive_poses[0].position.z <= 0.6 or obj.object.primitive_poses[0].position.x >= 1.5:
                continue
            # if obj.object.primitive_poses[0].position.x >= 0.8:
            #     # Remove detections that are far
            #     continue
            idx += 1
            obj.object.name = 'object_{}'.format(idx)
            # print(obj.object.name)
            if obj.object.name == "object_0":
                print(obj.object.primitive_poses[0])
            
            # rospy.loginfo(obj.object)
            obj_pose = PoseStamped()
            obj_pose.pose = obj.object.primitive_poses[0]
            obj_pose.header.frame_id = 'base_link'
            self.graspable_objs.append(GraspableObject(obj.object.name, obj_pose))
            # obs_msg = ObjectDetection()
            # transformed_pt = self.tf_listener.transformPose('map', obj_pose)
            # obs_msg.pose = transformed_pt.pose
            # obs_msg.label = 'bottle'
            # self.observation_pub.publish(obs_msg)
            # # dims: 0.0755453705788, 0.0773956924677, 0.17219388485
            # rospy.loginfo("Adding {} to planning scene.".format(obj.object.name))
            # rospy.loginfo("dim 1: {}".format(obj.object.primitives[0].dimensions[0]))
            # rospy.loginfo("dim 2: {}".format(obj.object.primitives[0].dimensions[1]))
            # rospy.loginfo("dim 3: {}".format(obj.object.primitives[0].dimensions[2]))
            try:
                self.scene.add_box(obj.object.name, obj_pose, (obj.object.primitives[0].dimensions[0], obj.object.primitives[0].dimensions[1], obj.object.primitives[0].dimensions[2]))
            except:
                rospy.logwarn("Basic perception could not get object dimensions. Defaulting to default values")
                self.scene.add_box(obj.object.name, obj_pose, (0.0755453705788, 0.0773956924677, 0.17219388485))
        idx = -1
        for surface in find_result.support_surfaces:
            if surface.primitive_poses[0].position.z <= 0.25:
                continue
            idx+=1
            surface.name = 'surface_{}'.format(idx)
            surface_pose = PoseStamped()
            surface_pose.pose = surface.primitive_poses[0]
            surface_pose.header.frame_id = 'base_link'
            rospy.logwarn("dims: {:.4f}, {:.4f}, {:.4f}".format(surface.primitives[0].dimensions[0], surface.primitives[0].dimensions[1], surface.primitives[0].dimensions[2]))
            surface_pose.pose.position.z /= 2
            self.scene.add_box(surface.name, surface_pose, (surface.primitives[0].dimensions[0], surface.primitives[0].dimensions[1], surface_pose.pose.position.z*2))
            self.arm.set_support_surface_name(surface.name)
    
    def grasploc_cb(self, msg):
        self.update_scene()
        rospy.sleep(5)
        self.send_grasps(msg.poses)
        


        


if __name__ == '__main__':
    rospy.init_node('fetch_pick')
    foo = GraspClient()
    rospy.logwarn("Starting spin")
    while not rospy.is_shutdown():
        rospy.spin()
    roscpp_shutdown(sys.argv)

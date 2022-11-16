#! /usr/bin/python

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from math import sin, cos
from fetch_actions.msg import MoveBaseRequestAction
from nav_msgs.srv import GetPlan, GetPlanResponse
from nav_msgs.msg import Path
import numpy as np

class MoveBase(object):
    def __init__(self):
        self.cur_pose = PoseStamped()
        self.server = actionlib.SimpleActionServer("kisailus_move_base", MoveBaseRequestAction, self.request_callback, auto_start=False)
        self.server.start()
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.cur_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_cb)
        # self.goal_sub = rospy.Subscriber("/move_base/to", Pose2D, self.callback)
        rospy.loginfo("MOVE BASE: Waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo("MOVE BASE: Connected to move_base!")
        self.cur_pose = PoseStamped()
        self.make_plan_client = rospy.ServiceProxy("move_base/make_plan", GetPlan)
        self.plan_pub = rospy.Publisher("kisailus_make_plan", Path)
    
    def request_callback(self, msg):
        rospy.loginfo("MOVE BASE: Received request...")
        self.plan_to(msg.x, msg.y, msg.theta)

        # self.go_to(msg.x, msg.y, msg.theta)
        # rospy.loginfo("MOVE BASE: Action completed")
        self.server.set_succeeded()

    def plan_to(self, x, y, theta):
        if self.cur_pose == None:
            print("No pose")
            return
        start = self.cur_pose
        thetas = np.linspace(0, 2*np.pi, num=20)
        r = 0.8
        for t in thetas:
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = 'map'
            goal.pose.position.x = x + r*np.cos(t)
            goal.pose.position.y = y + r*np.sin(t)
            goal.pose.orientation.z = sin(theta/2.0)
            goal.pose.orientation.w = cos(theta/2.0)
            rospy.logwarn("Planning to {}".format(goal.pose))
            resp = self.make_plan_client(start, goal, 0.001)
            if len(resp.plan.poses) > 0:
                path = Path()
                path.header.frame_id = 'map'
                path.header.stamp = rospy.Time.now()
                path.poses = resp.plan.poses
                self.plan_pub.publish(path)
            else:
                rospy.logwarn("No valid path found!")
            

    def pose_cb(self, msg):
        rospy.logwarn("Got a pose")
        self.cur_pose.header = msg.header
        self.cur_pose.pose = msg.pose.pose


    def go_to(self, x, y, theta, frame='map'):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        self.client.send_goal(move_goal)
        self.client.wait_for_result()
        self.client.get_result()

if __name__ == '__main__':
    rospy.init_node('move_base_client')
    move_base = MoveBase()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
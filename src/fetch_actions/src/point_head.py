#! /usr/bin/python

import rospy
import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import Point
from fetch_actions.msg import PointHeadRequestAction

class PointHeadClient(object):
    def __init__(self):
        self.server = actionlib.SimpleActionServer("kisailus_point_head", PointHeadRequestAction, self.callback, auto_start=False)
        self.server.start()
        self.client = actionlib.SimpleActionClient('head_controller/point_head', PointHeadAction)
        self.look_sub = rospy.Subscriber('/point_head/at', Point, self.callback)
        rospy.loginfo("POINT HEAD: Waiting for head_controller...")
        self.client.wait_for_server()
        rospy.loginfo("POINT HEAD: Connected to head_controller!")

    def callback(self, msg):
        rospy.loginfo("POINT HEAD: Received request")
        self.look_at(msg.x, msg.y, msg.z)
        rospy.loginfo("POINT HEAD: Action completed")
        self.server.set_succeeded()
        
    def look_at(self, x, y, z, frame='map'):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(1.0)
        self.client.send_goal(goal)
        self.client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('point_head_client')
    point_head = PointHeadClient()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
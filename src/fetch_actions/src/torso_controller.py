#! /usr/bin/python

import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float32
from fetch_actions.msg import TorsoControlRequestAction

class TorsoControllerClient(object):
    def __init__(self):
        self.server = actionlib.SimpleActionServer("kisailus_torso_controller", TorsoControlRequestAction, self.callback, auto_start=False)
        self.server.start()
        self.client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        # self.pos_sub = rospy.Subscriber("move_torso/to", Float32, self.callback)
        self.joint_names = ['torso_lift_joint']
        rospy.loginfo("TORSO CONTROLLER: Waiting for torso_controller")
        self.client.wait_for_server()
        rospy.loginfo("TORSO CONTROLLER: Connected to server ...")
        
    
    def callback(self, msg):
        rospy.loginfo("TORSO CONTROLLER: Received request...")
        self.move_to([msg.height])
        rospy.loginfo("TORSO CONTROLLER: Action completed")
        self.server.set_succeeded()
    
    def move_to(self, position):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points.append(JointTrajectoryPoint())
        traj.points[0].positions = position
        traj.points[0].velocities = [0.0]
        traj.points[0].accelerations = [0.0]
        traj.points[0].time_from_start = rospy.Duration(5.0)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = traj
        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('torso_client')
    torso_action = TorsoControllerClient()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
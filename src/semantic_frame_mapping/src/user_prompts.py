#!/usr/bin/python3
import rospy
from std_msgs.msg import String


rospy.init_node("escape_room")

pub = rospy.Publisher("/escape_room/input", String, queue_size=10)
msg = String()
running = True

first_right = False
while not first_right:
    first_a = str(input("What is the weather? "))
    if first_a == 'hot':
        msg.data = "finesse"
        pub.publish(msg)
        first_right = True
    else:
        print("Not the right answer... try again")

second_right = False
while not second_right:
    second_a = str(input("Do you love robots? "))
    if second_a == 'yes':
        msg.data = 'dat way'
        pub.publish(msg)
        second_right = True
    else:
        print("not the right answer... try again")

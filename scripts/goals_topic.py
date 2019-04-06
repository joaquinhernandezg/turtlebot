#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def talker(list):
    pub = rospy.Publisher('list_goals', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    while not rospy.is_shutdown():
        if len(list):
            pub.publish(list.pop())


def pose_from_coordinates(x, y, theta):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0

    pose.orientation = quaternion_from_euler(0, 0, theta)


    return pose

def send_list(list):
    try:
        talker(list)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    lista = [(0,1,pi/2), (0,0, pi)]
    list = [pose_from_coordinates(*x) for x in lista]
    send_list(list)

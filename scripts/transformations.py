from math import sin, asin, cos, acos, tan, atan, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose

def angle_to_zero_2pi(angle):
    new_angle = asin(sin(angle))
    if cos(angle) < 0:
        return pi-new_angle
    else:
        if new_angle < 0:
            return 2*pi + new_angle
    return new_angle

def angle_to_pi_minuspi(angle):
    new_angle = acos(cos(angle))
    if sin(angle) < 0:
        return -new_angle
    return new_angle

def pose_from_coordinates(x, y, theta):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0
    pose.orientation = quaternion_from_euler(0, 0, theta)

    return pose

def point_to_point_angle(pose, x2, y2):
    print pose.position, x2, y2


    x1 = pose.position.x
    y1 = pose.position.y
    print pose.position, x2, y2

    if x1 > x2:
        print 1
        if y1 > y2:
            print 1
            angle = atan((y1-y2)/(x1-x2))
        elif y2 > y1:
            print 2
            angle = -(atan((y1-y2)/(x1-x2)))
        else:
            print 3
            angle = 0
    elif x2 > x1:
        print 2
        if y1 > y2:
            angle = pi - atan((y1-y2)/(x2-x1))
        elif y2 > y1:
            angle = -(atan((y1-y2)/(x1-x2)) + pi/2)
        else:
            angle = pi
    else:
        print 3
        if y1 > y2:
            angle = pi/2
        elif y2 > y1:
            angle = -pi/2
        else:
            print x1, y1, x2, y2
            return None
    angle = angle_to_zero_2pi(angle)
    return angle

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from math import sin,asin, cos,acos, tan, atan, sqrt, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time

VEL_TOPIC = "/cmd_vel_mux/input/teleop"
ANG_VEL = 0.5
LIN_VEL = 1

class MoveRobot( object ):
    def __init__(self):
        self.robot_x = 0
        self.robot_y = 0
        self.robot_angle_ = 0
        rospy.init_node("accion_mover", anonymous=False)
        self.pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size = 10)
        #rospy.Subscriber("/odom", Odometry, self.print_odom)
        #sub = rospy.Subscriber('lista_goals', Pose, self.mover_robot_goal)
        self.vel = Twist()

        #rospy.spin()
        #self.mover_robot_goal([(1,0,pi/2),(1,1,-pi), (0,1, -pi/2), (0,0,0)])
        #self.mover_robot_goal([(0,3,-pi/2),(0,0,pi/2)])
        self.mover_robot_goal([(3,0,pi),(0,0,0)])

    @property
    def robot_angle(self):
        return self.robot_angle_

    @robot_angle.setter
    def robot_angle(self, value):
        angle = value + self.robot_angle_

        if 2*pi-value < 0.05:
            print value
            print 2*pi
            self.robot_angle_ = 0
            return
        self.robot_angle_ = self.angle_to_zero_2pi(value)

    def print_odom(self, odom):
        rospy.sleep(1)
        print odom

    def angle_to_zero_2pi(self, angle):
        new_angle = asin(sin(angle))
        if cos(angle) < 0:
            return pi-new_angle
        else:
            if new_angle < 0:
                return 2*pi + new_angle
        return new_angle

    def angle_to_pi_minuspi(self, angle):
        new_angle = acos(cos(angle))
        if sin(angle) < 0:
            return -new_angle
        return new_angle


    def pose_from_coordinates(self, x, y, theta):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0

        pose.orientation = quaternion_from_euler(0, 0, theta)

        return pose

    def linea_recta(self, lin_vel, time_):
        rate = rospy.Rate(20)
        end_time = time.time()+time_


        self.vel.linear.x = lin_vel
        self.vel.linear.y = 0
        self.vel.linear.z = 0

        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0


        while not rospy.is_shutdown() and time.time()<end_time:
            self.pub.publish(self.vel)
            rate.sleep()

        self.vel.linear.x = 0
        self.pub.publish(self.vel)

        total_distance = lin_vel*time_
        self.robot_x += total_distance*cos(self.robot_angle)
        self.robot_y += total_distance*sin(self.robot_angle)
        rospy.loginfo("Robot reached destination")
        rospy.loginfo("Position: x:%f y:%f",self.robot_x, self.robot_y)


    def orientar(self, ang_vel, delta_theta):
        if delta_theta < 0:
            ang_vel = -ang_vel

        time_ = delta_theta/ang_vel
        rate_hz = 10
        rate = rospy.Rate(rate_hz)
        end_time = time.time()+time_

        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0

        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = ang_vel

        while not rospy.is_shutdown() and time.time()<end_time:

            self.pub.publish(self.vel)

            rate.sleep()

        self.vel.angular.z = 0
        self.pub.publish(self.vel)

        self.robot_angle = round(ang_vel*time_ +self.robot_angle, 2)

        rospy.loginfo("Robot rotated %f ",ang_vel*time_)
        rospy.loginfo("Robot angle %f ",self.robot_angle)

    def point_to_point_angle(self, pose):
        x1, x2 = (round(pose.position.x, 2), round(self.robot_x, 2))
        y1, y2 = (round(pose.position.y, 2), round(self.robot_y, 2 ))
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
                return None
        angle = self.angle_to_zero_2pi(angle)

        rospy.loginfo("ROBOT IS IN X: %f- Y: %f and Theta: %f", self.robot_x, self.robot_y, self.robot_angle)
        rospy.loginfo("ROBOT GOES TO X: %f- Y: %f and Theta: %f", x1, y1, self.angle_to_zero_2pi(euler_from_quaternion(pose.orientation)[2]))
        rospy.loginfo("Angle go to is: %f", angle)


        return angle

    def mover_a_pose(self, pose):
        global ANG_VEL, LIN_VEL

        delta_theta = self.angle_to_pi_minuspi(self.point_to_point_angle(pose)-self.robot_angle)
        self.orientar(ANG_VEL, delta_theta)
        rospy.loginfo("delta theta: %f ", delta_theta )

        rospy.sleep(1)
        distance = sqrt((pose.position.y-self.robot_y)**2+(pose.position.x-self.robot_x)**2)
        time_to_traslate = distance/LIN_VEL
        self.linea_recta(LIN_VEL, time_to_traslate)

        rospy.sleep(1)
        delta_theta = self.angle_to_pi_minuspi(euler_from_quaternion(pose.orientation)[2]-self.robot_angle)
        self.orientar(ANG_VEL, delta_theta)
        rospy.loginfo("FINAL ANGLE: %f", self.robot_angle)

        rospy.sleep(0.1)

    def mover_robot_goal(self, lista):
        for p in lista:
            #self.mover_a_pose(p)
            self.mover_a_pose(self.pose_from_coordinates(*p))


if __name__ == "__main__":
    MoveRobot()

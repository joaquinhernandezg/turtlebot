#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class Detector():
    def __init__(self, webcam):
        rospy.init_node("camera")
        self.image = None
        self.args = {'filter': 'RGB', 'webcam': webcam}

        self.sub = rospy.Subscriber('/camera/rgb/image_color', Image, self.image_saver)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_saver)

        self.pub = rospy.Publisher('/object_centroid', Point, queue_size = 10)

        self.bridge = CvBridge()

        self.main()

    def callback(self, data):
        pass
    def image_saver(data):
        self.image = data

    def depth_saver(data):
        depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
        depth_array = np.array(depth_image, dtype=np.float32)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        self.depth_array = depth_array

    def setup_trackbars(self, range_filter):
        cv2.namedWindow("Trackbars", 0)

        for i in ["MIN", "MAX"]:
            v = 0 if i == "MIN" else 255

            for j in range_filter:
                cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, self.callback)

    def get_trackbar_values(self, range_filter):
        values = []

        for i in ["MIN", "MAX"]:
            for j in range_filter:
                v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
                values.append(v)
        return values

    def main(self):
        range_filter = self.args['filter'].upper()

        camera = cv2.VideoCapture(0)

        self.setup_trackbars(range_filter)

        while True:
            if self.args['webcam']:
                ret, image = camera.read()

                if not ret:
                    break

                if range_filter == 'RGB':
                    frame_to_thresh = image.copy()
                else:
                    frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            else:
                if self.image:
                    image = self.image.copy()
                    image = cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
                    frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                else:
                    continue


            v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = self.get_trackbar_values(range_filter)

            thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # find contours in the mask and initialize the current
            # (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None

            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if radius > 10:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(image, center, 3, (0, 0, 255), -1)
                    cv2.putText(image, "centroid", (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255),
                                1)
                    cv2.putText(image, "(" + str(center[0]) + "," + str(center[1]) + ")", (center[0] + 10, center[1] + 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                    point = Point()
                    point.x = x
                    point.y = y
                    self.pub.publish()

            # show the frame to our screen
            cv2.imshow("Original", image)
            cv2.imshow("Thresh", thresh)
            cv2.imshow("Mask", mask)

            if cv2.waitKey(1) & 0xFF is ord('q'):
                break

if __name__ == '__main__':
    Detector(True)

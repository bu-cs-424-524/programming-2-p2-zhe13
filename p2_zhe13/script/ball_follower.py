#!/usr/bin/env python3
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class BallFollower:
    def __init__(self):
        rospy.init_node('ball_follower')

        self.bridge = CvBridge()

        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/color/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        self.debug_view = rospy.get_param('~debug_view', True)

        rospy.Subscriber(self.rgb_topic, Image, self.rgb_callback, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)

        self.cmd_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

        self.rgb_image = None
        self.depth_image = None

        self.target_cx = None
        self.target_cy = None
        self.target_radius = None

        self.desired_distance = 1.0
        self.k_ang = 0.0025
        self.k_lin = 0.3

    def rgb_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.rgb_image = frame
            self.detect_red_ball(frame)
        except Exception as e:
            rospy.logwarn(f"RGB callback error: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logwarn(f"Depth callback error: {e}")

    def detect_red_ball(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # red + pink
        lower_red1 = np.array([0, 100, 70])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 100, 70])
        upper_red2 = np.array([179, 255, 255])

        lower_pink = np.array([145, 60, 80])
        upper_pink = np.array([169, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask3 = cv2.inRange(hsv, lower_pink, upper_pink)

        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.bitwise_or(mask, mask3)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.target_cx = None
        self.target_cy = None
        self.target_radius = None

        if len(contours) == 0:
            if self.debug_view:
                self.show_debug(frame, mask)
            return

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < 100:
            if self.debug_view:
                self.show_debug(frame, mask)
            return

        ((x, y), radius) = cv2.minEnclosingCircle(largest)
        M = cv2.moments(largest)

        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            self.target_cx = cx
            self.target_cy = cy
            self.target_radius = radius

            if self.debug_view:
                debug = frame.copy()
                cv2.circle(debug, (cx, cy), int(radius), (0, 255, 0), 2)
                cv2.circle(debug, (cx, cy), 4, (255, 0, 0), -1)
                self.show_debug(debug, mask)

    def show_debug(self, frame, mask):
        cv2.imshow("rgb_debug", frame)
        cv2.imshow("mask_debug", mask)
        cv2.waitKey(1)

    def get_depth(self):
        if self.depth_image is None or self.target_cx is None or self.target_cy is None:
            return None

        h, w = self.depth_image.shape[:2]
        cx = int(np.clip(self.target_cx, 0, w - 1))
        cy = int(np.clip(self.target_cy, 0, h - 1))

        window = self.depth_image[max(0, cy - 2):min(h, cy + 3),
                                  max(0, cx - 2):min(w, cx + 3)]

        vals = window[np.isfinite(window)]

        if len(vals) == 0:
            return None

        depth = float(np.median(vals))

        if depth > 10.0:
            depth /= 1000.0

        if depth <= 0.0 or depth > 10.0:
            return None

        return depth

    def control_once(self):
        cmd = Twist()

        if self.rgb_image is None:
            self.cmd_pub.publish(cmd)
            return

        if self.target_cx is None:
            cmd.angular.z = 0.2
            cmd.linear.x = 0.0
            self.cmd_pub.publish(cmd)
            return

        img_h, img_w = self.rgb_image.shape[:2]
        center_x = img_w / 2.0
        error_x = self.target_cx - center_x

        depth = self.get_depth()

        cmd.angular.z = -self.k_ang * error_x
        cmd.angular.z = np.clip(cmd.angular.z, -0.8, 0.8)

        if depth is None:
            cmd.linear.x = 0.0
            self.cmd_pub.publish(cmd)
            return

        error_d = depth - self.desired_distance

        if abs(error_x) < 120:
            cmd.linear.x = self.k_lin * error_d
            cmd.linear.x = np.clip(cmd.linear.x, -0.15, 0.15)
        else:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

        rospy.loginfo_throttle(
            1.0,
            f"ball=({self.target_cx},{self.target_cy}), depth={depth:.2f} m, "
            f"vx={cmd.linear.x:.2f}, wz={cmd.angular.z:.2f}"
        )

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.control_once()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = BallFollower()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except:
            pass

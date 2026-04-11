#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_nav')

        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # masured data of L1/L2/L3
        self.L1 = (-0.240, -0.165, -0.636)
        self.L2 = (2.533, -14.772, -1.401)
        self.L3 = (3.997, -24.274, -1.436)

        self.waypoints = [self.L2, self.L3, self.L1]

        self.pos_tol = rospy.get_param('~pos_tol', 0.30)
        self.yaw_tol = rospy.get_param('~yaw_tol', 0.35)
        self.pause_after_arrival = rospy.get_param('~pause_after_arrival', 2.0)

        rospy.loginfo("Waiting for AMCL pose...")
        while not rospy.is_shutdown() and self.current_x is None:
            rospy.sleep(0.2)

        rospy.loginfo("AMCL pose received. Starting waypoint navigation.")
        rospy.sleep(1.0)

    def amcl_callback(self, msg):
        pose = msg.pose.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y
        q = pose.orientation
        self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def publish_goal(self, x, y, yaw):
        q = quaternion_from_euler(0.0, 0.0, yaw)

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        rospy.loginfo(f"Publishing goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
        self.goal_pub.publish(goal)

    def reached_goal(self, gx, gy, gyaw):
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return False

        dist = math.hypot(self.current_x - gx, self.current_y - gy)
        yaw_err = abs(self.normalize_angle(self.current_yaw - gyaw))
        return (dist < self.pos_tol) and (yaw_err < self.yaw_tol)

    def run(self):
        rate = rospy.Rate(5)

        for i, wp in enumerate(self.waypoints):
            gx, gy, gyaw = wp

            for _ in range(3):
                self.publish_goal(gx, gy, gyaw)
                rospy.sleep(0.3)

            rospy.loginfo(f"Moving to waypoint {i+1}/{len(self.waypoints)}")

            while not rospy.is_shutdown():
                if self.reached_goal(gx, gy, gyaw):
                    rospy.loginfo(f"Reached waypoint {i+1}")
                    rospy.sleep(self.pause_after_arrival)
                    break
                rate.sleep()

        rospy.loginfo("Route finished: L1 -> L2 -> L3 -> L1")

if __name__ == '__main__':
    try:
        node = WaypointNavigator()
        node.run()
    except rospy.ROSInterruptException:
        pass

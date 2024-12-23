#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import quaternion_matrix

class OdomPublisher:
    def __init__(self, alpha=0.2):
        self.last_pose = None
        self.last_time = None

        self.alpha = alpha  # Low-pass filter coefficient (0 < alpha <= 1)
        self.filtered_velocity = np.array([0.0, 0.0, 0.0])  # Initial filtered velocity

        self.velocity_pub = rospy.Publisher("car_odom", Odometry, queue_size=1)
        rospy.Subscriber("car_pose", PoseStamped, self.pose_callback)

    def pose_callback(self, msg):
        current_time = rospy.Time.now()
        
        # If we have a previous pose, calculate velocity
        if self.last_pose is not None and self.last_time is not None:
            dt = (current_time - self.last_time).to_sec()
            if dt > 0:
                # Global frame velocity
                dx = msg.pose.position.x - self.last_pose.pose.position.x
                dy = msg.pose.position.y - self.last_pose.pose.position.y
                dz = msg.pose.position.z - self.last_pose.pose.position.z

                velocity_global = np.array([dx / dt, dy / dt, dz / dt])

                # Convert velocity to body frame
                orientation = msg.pose.orientation
                quat = [orientation.x, orientation.y, orientation.z, orientation.w]
                rotation_matrix = quaternion_matrix(quat)[:3, :3]
                velocity_body = np.dot(rotation_matrix.T, velocity_global)

                # Apply low-pass filter
                self.filtered_velocity = (
                    self.alpha * velocity_body + (1 - self.alpha) * self.filtered_velocity
                )

                # Publish the filtered velocity
                twist_msg = Twist()
                twist_msg.linear.x = self.filtered_velocity[0]
                twist_msg.linear.y = self.filtered_velocity[1]
                twist_msg.linear.z = self.filtered_velocity[2]

                twist_covar_msg = TwistWithCovariance()
                twist_covar_msg.twist = twist_msg
                pos_covar_msg = PoseWithCovariance()
                pos_covar_msg.pose = msg.pose
                odom_msg = Odometry()
                odom_msg.pose = pos_covar_msg
                odom_msg.twist = twist_covar_msg
                self.velocity_pub.publish(odom_msg)

        # Update the last pose and time
        self.last_pose = msg
        self.last_time = current_time


if __name__ == "__main__":
    rospy.init_node("odom_publisher")

    # Get alpha parameter for the filter, default is 0.1
    alpha = rospy.get_param("~alpha", 0.2)
    OdomPublisher(alpha)
    rospy.spin()
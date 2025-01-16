#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
from tf.transformations import quaternion_matrix, euler_from_quaternion, quaternion_from_euler

class OdomPublisher:
    def __init__(self, alpha=0.2):
        self.last_pose = None
        self.last_time = None

        self.alpha = alpha  # Low-pass filter coefficient (0 < alpha <= 1)
        self.filtered_velocity = np.array([0.0, 0.0, 0.0])  # Initial filtered velocity

        self.last_publish = rospy.Time.now().to_sec()
        self.pub_rate = 100.0

        self.odom_pub = rospy.Publisher("car_odom", Odometry, queue_size=1)
        self.pose_pub = rospy.Publisher("car_pose", PoseStamped, queue_size=1)
        
        rospy.Subscriber("mocap_pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("imu/data", Imu, self.imu_callback)

        self.imu_ori = [0.0,0.0,0.0]
        self.odom_msg = Odometry()
        self.pose_msg = PoseStamped()
        self.update_odom = False
        self.main_loop()

    def main_loop(self):
        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            if self.update_odom:
                self.odom_pub.publish(self.odom_msg)
                self.pose_pub.publish(self.pose_msg)
                self.update_odom = False
            rate.sleep()

    def imu_callback(self, msg):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        self.imu_ori = euler_from_quaternion(quaternion)
    
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
                quaternion = (
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w
                )
                #use imu for roll and pitch
                rpy = euler_from_quaternion(quaternion)
                ori = [0,0,0]
                ori[0] = self.imu_ori[0]
                ori[1] = self.imu_ori[1]
                ori[2] = rpy[2]

                quat = quaternion_from_euler(*ori, 'sxyz')
                msg.pose.orientation.x = quat[0]
                msg.pose.orientation.y = quat[1]
                msg.pose.orientation.z = quat[2]
                msg.pose.orientation.w = quat[3]

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

                timestamp = msg.header.stamp

                # Update the timestamp for each message
                self.odom_msg.header.stamp = timestamp
                self.odom_msg.header.frame_id = "map"  # or any relevant frame_id
                self.odom_msg.child_frame_id = "base_link"
                self.pose_msg.header.stamp = timestamp
                self.pose_msg.header.frame_id = "map"  # or any relevant frame_id
                self.pose_msg.pose = msg.pose

                twist_covar_msg = TwistWithCovariance()
                twist_covar_msg.twist = twist_msg
                pos_covar_msg = PoseWithCovariance()
                pos_covar_msg.pose = msg.pose

                self.odom_msg.pose = pos_covar_msg
                self.odom_msg.twist = twist_covar_msg
                self.update_odom = True

        # Update the last pose and time
        self.last_pose = msg
        self.last_time = current_time


if __name__ == "__main__":
    rospy.init_node("odom_publisher")

    # Get alpha parameter for the filter, default is 0.1
    alpha = rospy.get_param("~alpha", 0.2)
    OdomPublisher(alpha)
    rospy.spin()
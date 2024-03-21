#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, PoseWithCovarianceStamped, PoseWithCovariance


if __name__ == "__main__":
    import sys

    car_name = sys.argv[1]

    pub_topic = "initialpose"
    sub_topic = "mocap_pose"

    publisher = rospy.Publisher(pub_topic, PoseWithCovarianceStamped, queue_size=1)

    rospy.init_node("pose_setter")

    rate = rospy.Rate(10.0)

    for _ in range(6):
        msg = rospy.wait_for_message(sub_topic, PoseStamped, timeout=None)
        p = PoseWithCovarianceStamped()
        p.header = msg.header
        p.pose = PoseWithCovariance()
        p.pose.pose = msg.pose
        publisher.publish(p)

        rate.sleep()
    rospy.logwarn("Sent initial pose for {}:".format(car_name))

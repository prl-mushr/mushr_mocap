#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, PoseWithCovarianceStamped
import tf
from std_msgs.msg import Header, Float32

# This script publishes "/floor" topic to have correct xyz axis orientation.
if __name__ == '__main__':

    import sys

    car_name = sys.argv[1]

    rospy.init_node("floor_pose_publisher")

    publisher = rospy.Publisher("/floor", PoseStamped, queue_size=1)

    listener = tf.TransformListener()

    d = rospy.Duration(1.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform("/floor", "/vrpn_client_node/floor", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        p = PoseStamped()
        p.header = Header()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"
        p.pose.position.x = trans[2]
        p.pose.position.y = trans[0]
        p.pose.position.z = trans[1]s

        publisher.publish(p)
        rospy.sleep(d)

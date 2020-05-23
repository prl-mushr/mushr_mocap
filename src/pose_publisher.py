#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, PoseWithCovarianceStamped
import tf
from std_msgs.msg import Header, Float32

if __name__ == '__main__':

    import sys

    car_name = sys.argv[1]

    pub_topic = "mocap_pose"
    floor_tf = "floor"
    car_tf = car_name + "/mocap_pose"
    rospy.logfatal("car tf")
    rospy.init_node("pose_publisher")

    publisher = rospy.Publisher(pub_topic, PoseStamped, queue_size=1)

    listener = tf.TransformListener()

    rate = rospy.Rate(360.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(car_tf, floor_tf, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        p = PoseStamped()
        p.header = Header()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"
        p.pose.position.x = trans[0]
        p.pose.position.y = trans[1]
        p.pose.position.z = trans[2]

        p.pose.orientation.x = rot[0]
        p.pose.orientation.y = rot[1]
        p.pose.orientation.z = rot[2]
        p.pose.orientation.w = rot[3]

        publisher.publish(p)
        rate.sleep()

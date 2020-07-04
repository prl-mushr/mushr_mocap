#!/usr/bin/env python

# This publishes "absolute" pose of the car with respect to the "ground" reference set during calibration.
# Since mocap publishes y-up we swap the axis to be z-up and x to be the horizontal axis of the room.
from copy import deepcopy
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, PoseWithCovarianceStamped
import tf
from std_msgs.msg import Header, Float32

def publish_car_pose(msg):
    # The point on the top surface in the center of those 4 points has the following offset
    # w.r.t the base_link: (-0.058325, 0.0, 0.08125)

    p = deepcopy(msg)
    p.pose.position.x = msg.pose.position.z + 0.058325
    p.pose.position.y = msg.pose.position.x
    p.pose.position.z = msg.pose.position.y - 0.081250

    publisher.publish(p)


if __name__ == '__main__':

    import sys

    car_name = sys.argv[1]

    rospy.init_node("pose_publisher")

    pub_topic = "mocap_pose"
    publisher = rospy.Publisher(pub_topic, PoseStamped, queue_size=1)
    subscriber = rospy.Subscriber("/vrpn_client_node/{}/pose".format(car_name), PoseStamped, publish_car_pose)
    rospy.spin()
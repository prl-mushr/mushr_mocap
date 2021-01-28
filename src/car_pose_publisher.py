#!/usr/bin/env python

# This publishes "absolute" pose of the car with respect to the "ground" reference set during calibration.
# Since mocap publishes y-up we swap the axis to be z-up and x to be the horizontal axis of the room.
from copy import deepcopy
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, PoseWithCovarianceStamped
import tf
from std_msgs.msg import Header, Float32
from tf.transformations import *
import numpy as np

def publish_car_pose(msg):
    # The point on the top surface in the center of those 4 points has the following offset
    # w.r.t the base_link: (-0.058325, 0.0, 0.08125)

    # to matrix
    orientation = msg.pose.orientation
    quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    rotation = quaternion_matrix(quat)
  

    # rotate axes
    R = np.array([[0,   -1,  0],
                  [1,   0,  0],
                  [0,   0,  1]], dtype=np.float32)

    R2 = np.array([[0,  0, 1],
                  [1,   0,  0],
                  [0,   1,  0]], dtype=np.float32)

    R3 = np.array([[0,  -1,  0],
                   [1,  0,  0],
                   [0,  0,  1]], dtype=np.float32)
            

    #rotation[:3,:3] = np.dot(np.dot(R2, np.dot(rotation[:3,:3], R)), R3)
    #rotation[:3,:3] = np.dot(R3, np.dot(R2, np.dot(rotation[:3,:3], R)))
    rotation[:3,:3] = np.dot(rotation[:3,:3], R)
    
    q = quaternion_from_matrix(rotation)

    p = deepcopy(msg)
    p.pose.position.x = msg.pose.position.x + 0.058325
    p.pose.position.y = msg.pose.position.y
    p.pose.position.z = msg.pose.position.z - 0.081250

    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    publisher.publish(p)


if __name__ == '__main__':

    import sys

    car_name = sys.argv[1]

    rospy.init_node("pose_publisher")

    pub_topic = rospy.get_param(rospy.search_param("topic_name"))
    publisher = rospy.Publisher(pub_topic, PoseStamped, queue_size=1)
    subscriber = rospy.Subscriber("/vrpn_client_node/{}/pose".format(car_name), PoseStamped, publish_car_pose)
    rospy.spin()

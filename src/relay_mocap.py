#!/usr/bin/env python3

# This publishes "absolute" pose of the car with respect to the "ground" reference set during calibration.
# Since mocap publishes y-up we swap the axis to be z-up and x to be the horizontal axis of the room.
import rospy
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import yaml
import sys

config_path = "/root/catkin_ws/src/mushr_mocap/configs/mocap_tf_offset.yaml"
with open(config_path) as f:
    config = yaml.safe_load(f)

DEG2RAD = np.pi/180.0

off_x = config["x"]
off_y = config["y"] 
off_z = config["z"] 

off_roll = config["roll"]*DEG2RAD
off_pitch = config["pitch"]*DEG2RAD
off_yaw = config["yaw"]*DEG2RAD

def publish_car_pose(msg):
    global off_x, off_y, off_z, off_roll, off_pitch, off_yaw
    orientation = msg.pose.orientation
    quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    roll, pitch, yaw = euler_from_quaternion(quat)
    # hacky orientation offset (ideally you should have quaternion offsets, this sort of offset only works if you just want to rotate around 1 axis at a time)
    roll += off_roll
    pitch += off_pitch
    yaw += off_yaw

    quat = quaternion_from_euler(roll, pitch, yaw)

    p = deepcopy(msg)
    p.pose.position.x = msg.pose.position.x + off_x
    p.pose.position.y = msg.pose.position.y + off_y
    p.pose.position.z = msg.pose.position.z + off_z

    p.pose.orientation.x = quat[0]
    p.pose.orientation.y = quat[1]
    p.pose.orientation.z = quat[2]
    p.pose.orientation.w = quat[3]
    publisher.publish(p)


if __name__ == '__main__':
    rospy.init_node("pose_publisher")
    
    asset_name = sys.argv[1]
    car_name = sys.argv[2]
    
    publisher = rospy.Publisher('{}/mocap_pose'.format(car_name), PoseStamped, queue_size=1)
    subscriber = rospy.Subscriber("/vrpn_client_node/{}/pose".format(asset_name), PoseStamped, publish_car_pose)
    rospy.spin()

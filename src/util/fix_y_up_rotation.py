#!/usr/bin/env python

# Use this to fix rotations for mocap bags with y-up rotation
from copy import deepcopy
import argparse
from rosbag import Bag
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_matrix

# rotate axes
R = np.array([[0, 1, 0],
              [0, 0, 1],
              [1, 0, 0]], dtype=np.float32)

R2 = np.array([[0, 0, 1],
               [1, 0, 0],
               [0, 1, 0]], dtype=np.float32)

R3 = np.array([[0, -1, 0],
               [1, 0, 0],
               [0, 0, 1]], dtype=np.float32)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("-o", "--output", default="new_bag")
    parser.add_argument("--topic-to-fix", default="mocap_pose")
    args = parser.parse_args()

    # load bag
    with Bag(args.output, 'w') as f:
        for topic, msg, t in Bag(args.bag):
            if args.topic_to_fix in topic:
                msg_new = deepcopy(msg)

                orientation = msg.pose.orientation
                quat = [orientation.x, orientation.y, orientation.z, orientation.w]
                rotation = quaternion_matrix(quat)
                rotation[:3, :3] = np.dot(np.dot(R2, np.dot(rotation[:3, :3], R)), R3)

                q = quaternion_from_matrix(rotation)

                msg_new.pose.orientation.x = q[0]
                msg_new.pose.orientation.y = q[1]
                msg_new.pose.orientation.z = q[2]
                msg_new.pose.orientation.w = q[3]

                f.write(topic, msg_new, t)
            else:
                f.write(topic, msg, t)

import rospy

if __name__ == "__main__":
    import sys

    pub_topic = "initialpose"
    sub_topic = "mocap_pose"

    publisher = rospy.Publisher(pub_topic, PoseWithStamp, queue_size=1)

    try:
        rospy.init_node("pose_setter")
        rospy.wait_for_message(sub_topic, PoseWithStamp, timeout=None)
        publisher.pub(msg)

    except rospy.ROSInterruptException:
        pass

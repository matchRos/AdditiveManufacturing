#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Pose

class RemapUrEeTopic:
    def __init__(self) -> None:
        ns = rospy.get_param("~namespace", "mur620c/UR10_r/")
        self.pose = PoseStamped()
        self.pose.header.frame_id = ns+"base_link"

        self.pub = rospy.Publisher(ns + "ur_calibrated_pose", PoseStamped, queue_size=1)
        rospy.Subscriber(ns + "tcp_pose", Pose, self.cb_ee_pose)

    def cb_ee_pose(self, msg: Pose):
        self.pose.header.stamp = rospy.Time.now()
        self.pose.pose = msg
        self.pub.publish(self.pose)

if __name__ == "__main__":
    rospy.init_node("ur_calibrated_pose")
    RemapUrEeTopic()
    rospy.spin()
    
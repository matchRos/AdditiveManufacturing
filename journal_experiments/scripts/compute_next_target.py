#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf import transformations
from tf.broadcaster import TransformBroadcaster
import math


class ComputeNextTarget():

    def config(self):
        self.ur_target_velocity = 0.1

    def __init__(self):
        rospy.init_node("compute_next_target_node")
        self.config()
        self.mir_path_sub = rospy.Subscriber("/mir_path", Path, self.mir_path_callback)
        self.ur_path_sub = rospy.Subscriber("/ur_path", Path, self.ur_path_callback)
        
        self.mir_target_path = Path()
        self.ur_target_path = Path()
        self.current_mir_pose = PoseStamped()
        self.current_ur_pose = PoseStamped()

        self.mir_path_index = 0
        self.ur_path_index = 0



        self.tf_broadcaster = TransformBroadcaster()

        self.run()
        rospy.spin()


    def run(self):
        # wait for mir and ur path
        rospy.loginfo("Waiting for mir and ur path")
        rospy.wait_for_message("/mir_path", Path)
        rospy.wait_for_message("/ur_path", Path)
        rospy.loginfo("Received mir and ur path")

        # set current mir and ur pose to first pose in path
        self.current_mir_pose = self.mir_target_path.poses[0]
        self.current_ur_pose = self.ur_target_path.poses[0]

        # compute distance travelled by mir and ur
        self.ur_distance_travelled = self.compute_distance_travelled(self.ur_target_path)
        self.mir_distance_travelled = self.compute_distance_travelled(self.mir_target_path)

        print(self.ur_distance_travelled)

        #self.compute_next_target()


    def compute_next_target(self):
        # check length of mir and ur path
        self.check_path_length()

        # get mir and ur target based on path index
        self.get_target_from_path()

        # check if ur target is reached in this iteration and update ur path index
        self.check_target_reached()

        # compute mir velocity to reach next target
        self.compute_mir_velocity()

        # broadcast next target
        self.broadcast_next_target()

    def compute_distance_travelled(self, path):
        distance = []
        for i in range(len(path.poses)-1):
            distance.append(math.sqrt((path.poses[i].pose.position.x - path.poses[i+1].pose.position.x)**2 + (path.poses[i].pose.position.y - path.poses[i+1].pose.position.y)**2))
        return distance


    def check_target_reached(self):
        pass


    def get_target_from_path(self):
        self.mir_target = self.mir_target_path.poses[self.mir_path_index]
        self.ur_target = self.ur_target_path.poses[self.ur_path_index]

    def compute_mir_velocity(self):
        pass
    
    def check_path_length(self):
        if len(self.mir_target_path.poses) != len(self.ur_target_path.poses):
            rospy.logerr("Length of mir and ur path do not match")
            return
        else:
            rospy.loginfo("Length of mir and ur path match and is: " + str(len(self.mir_target_path.poses)))

    def broadcast_next_target(self):
        rate = rospy.Rate(10)
        for i in range(len(self.mir_target_path.poses)):
            self.tf_broadcaster.sendTransform((self.mir_target_path.poses[i].pose.position.x, self.mir_target_path.poses[i].pose.position.y, self.mir_target_path.poses[i].pose.position.z),
                                              (self.mir_target_path.poses[i].pose.orientation.x, self.mir_target_path.poses[i].pose.orientation.y, self.mir_target_path.poses[i].pose.orientation.z, self.mir_target_path.poses[i].pose.orientation.w),
                                              rospy.Time.now(),
                                              "mir_target",
                                              "map")
            self.tf_broadcaster.sendTransform((self.ur_target_path.poses[i].pose.position.x, self.ur_target_path.poses[i].pose.position.y, self.ur_target_path.poses[i].pose.position.z),
                                              (self.ur_target_path.poses[i].pose.orientation.x, self.ur_target_path.poses[i].pose.orientation.y, self.ur_target_path.poses[i].pose.orientation.z, self.ur_target_path.poses[i].pose.orientation.w),
                                              rospy.Time.now(),
                                              "ur_target",
                                              "map")
            rate.sleep()

    def mir_path_callback(self, path):
        self.mir_target_path = path

    def ur_path_callback(self, path):
        self.ur_target_path = path

if __name__ == '__main__':
    ComputeNextTarget()
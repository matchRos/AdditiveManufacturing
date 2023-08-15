#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import tf
import numpy as np
import math

class Create_path():

    def config(self):
        self.start_pose.position.x = -2.5
        self.target_pose.position.x = 2.5
        self.point_per_meter = 100
        self.dist = 100
        self.r_ur = 1.0
        self.r_mir=2.0
        self.shift_ur = 0.1 # in meter
        pass
    
    def main(self):
        rospy.init_node("create_path_node")
        
        self.start_pose = Pose()
        self.target_pose = Pose()
        self.config()
        rospy.sleep(0.1)
        self.path_pub = rospy.Publisher("/mir_path", Path, queue_size=1)
        rospy.sleep(0.1)
        self.path_pub_ur = rospy.Publisher("/ur_path", Path, queue_size=1)
        
        rospy.sleep(0.1)
        
        # create path between two points
        #dist = self.target_pose.position.x - self.start_pose.position.x
        path = Path()
        path.header.frame_id = "map"
        
        ur_path = Path()
        ur_path.header.frame_id = "map"
        for i in range(0,int(self.dist*self.point_per_meter)):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.header.seq = i
            # plan circle
            pose.pose.position.x = self.r_mir*math.cos(i/self.point_per_meter)
            pose.pose.position.y = self.r_mir*math.sin(i/self.point_per_meter)
            next_pose = Pose()
            next_pose.position.x = self.r_mir*math.cos((i+1)/self.point_per_meter)
            next_pose.position.y = self.r_mir*math.sin((i+1)/self.point_per_meter)
            orientation = math.atan2(next_pose.position.y - pose.pose.position.y, next_pose.position.x - pose.pose.position.x)
            q = tf.transformations.quaternion_from_euler(0, 0, orientation)

            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            #pose.pose.position.x = self.start_pose.position.x + i/self.point_per_meter
            #pose.pose.position.y = pose.pose.position.x*0.5 + 0.5 * math.sin(pose.pose.position.x)
            path.poses.append(pose)
            
            
            ur_pose = PoseStamped()
            ur_pose.header.frame_id = "map"
            ur_pose.header.stamp = rospy.Time.now()
            ur_pose.header.seq = i
            ur_pose.pose.position.x = self.r_ur*math.cos((i+self.shift_ur)/self.point_per_meter) + 0.05 * math.sin(6*i/self.point_per_meter)
            ur_pose.pose.position.y = self.r_ur*math.sin((i+self.shift_ur)/self.point_per_meter)  + 0.05 * math.cos(6*i/self.point_per_meter)
            ur_pose.pose.position.z = 0.5

            # rotate around x so that the gripper is pointing down
            q_rot = tf.transformations.quaternion_from_euler(np.pi, 0, 0)
            q_ur=tf.transformations.quaternion_multiply(q_rot, q)
            q_rot = tf.transformations.quaternion_from_euler(0, 0, np.pi/2)
            q_ur=tf.transformations.quaternion_multiply(q_rot, q_ur)

            ur_pose.pose.orientation.x = q_ur[0]
            ur_pose.pose.orientation.y = q_ur[1]
            ur_pose.pose.orientation.z = q_ur[2]
            ur_pose.pose.orientation.w = q_ur[3]
            
            ur_path.poses.append(ur_pose)
        
        rospy.sleep(1.1)
        rospy.loginfo("publishing MirPath")
        self.path_pub.publish(path)
        # rospy.wait_for_message("/mir_path", Path)
        rospy.sleep(1.1)
        rospy.loginfo("publishing URPath")
        self.path_pub_ur.publish(ur_path)
        # rospy.wait_for_message("/ur_path", Path)
        rospy.loginfo("paths published")
        rospy.sleep(1.1)
        print("path published")
        
        
        
if __name__=="__main__":
    exe = Create_path()
    exe.main()
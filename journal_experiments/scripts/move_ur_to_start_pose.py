#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import Int32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
import math
from tf import transformations

class MoveURToStartPose():
    
    def config(self):
        self.ur_velocity_limit = rospy.get_param("~ur_velocity_limit", 0.06)
        self.ur_angular_velocity_limit = rospy.get_param("~ur_angular_velocity_limit", 0.1)
        self.ur_acceleration_limit = rospy.get_param("~ur_acceleration_limit", 0.02)
        self.ur_angular_acceleration_limit = rospy.get_param("~ur_angular_acceleration_limit", 0.1)
        self.Kpx = rospy.get_param("~Kpx", 0.3)
        self.Kpy = rospy.get_param("~Kpy", 0.3)
        self.Kpz = rospy.get_param("~Kpz", 0.1)
        self.ur_target_tolerance = rospy.get_param("~ur_target_tolerance", 0.01)
        self.ur_scanner_angular_offset = rospy.get_param("~ur_scanner_angular_offset", 0.995 - math.pi/4)
        pass
    
    
    def __init__(self):
        rospy.init_node("control_ur_node")
        
        self.ur_command = Twist()
        self.ur_command_old = Twist()
        ur_start_pose_array = rospy.get_param("~ur_start_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.ur_start_pose = Pose()
        self.ur_start_pose.position.x = ur_start_pose_array[0]
        self.ur_start_pose.position.y = ur_start_pose_array[1]
        self.ur_start_pose.position.z = ur_start_pose_array[2]
        self.ur_start_pose.orientation.x = ur_start_pose_array[3]
        self.ur_start_pose.orientation.y = ur_start_pose_array[4]   
        self.ur_start_pose.orientation.z = ur_start_pose_array[5]
        self.ur_start_pose.orientation.w = ur_start_pose_array[6]
        
        print(self.ur_start_pose)
        
        self.ur_command_topic = rospy.get_param("~ur_command_topic", "/mur620c/UR10_r/twist_controller/command_safe")
        self.ur_pose_topic = rospy.get_param("~ur_pose_topic", "/mur620c/UR10_r/ur_calibrated_pose")
        self.ur_base_link_frame_id = rospy.get_param("~ur_base_link_frame_id", "mur620c/UR10_r/base_link")
        self.ur_twist_publisher = rospy.Publisher(self.ur_command_topic, Twist, queue_size=1)
        self.ur_target_pose_broadcaster = tf.TransformBroadcaster()
        
        rospy.Subscriber(self.ur_pose_topic, PoseStamped, self.ur_pose_callback)
        
        self.config()
        

    def move_ur_to_start_pose(self):
                
        # wait until ur_pose is published
        rospy.wait_for_message(self.ur_pose_topic, PoseStamped)
                
        rate = rospy.Rate(100)
        
        while not rospy.is_shutdown():
             # broadcast start pose
            self.broadcast_target_pose(self.ur_start_pose)
        
            # the ur is mounted backwards, so we have to invert the x and y axis
            ur_start_pose_x = - self.ur_start_pose.position.x
            ur_start_pose_y = - self.ur_start_pose.position.y
            
            # compute ur phi angle
            ur_current_phi = transformations.euler_from_quaternion([self.ur_pose_current.orientation.x, self.ur_pose_current.orientation.y, self.ur_pose_current.orientation.z, self.ur_pose_current.orientation.w])[2]
            ur_target_phi = transformations.euler_from_quaternion([self.ur_start_pose.orientation.x, self.ur_start_pose.orientation.y, self.ur_start_pose.orientation.z, self.ur_start_pose.orientation.w])[2]
        
            # calculate error
            e_x = ur_start_pose_x - self.ur_pose_current.position.x
            e_y = ur_start_pose_y - self.ur_pose_current.position.y
            e_z = self.ur_start_pose.position.z - self.ur_pose_current.position.z
            e_phi = ur_target_phi - ur_current_phi + self.ur_scanner_angular_offset
            
            if e_phi > math.pi:
                e_phi = e_phi - 2 * math.pi
            elif e_phi < -math.pi:
                e_phi = e_phi + 2 * math.pi
            print("e_phi: " + str(e_phi))
            #print(ur_target_phi + self.ur_scanner_angular_offset,ur_current_phi)
            
            # calculate command
            self.ur_command.linear.x = e_x * self.Kpx
            self.ur_command.linear.y = e_y * self.Kpy
            self.ur_command.linear.z = e_z * self.Kpz
            self.ur_command.angular.z = e_phi * self.Kpz
                       
            # limit velocity
            ur_command = self.limit_velocity(self.ur_command, self.ur_command_old)
            self.ur_command_old = ur_command
                       
            self.ur_twist_publisher.publish(ur_command)
        
            # check if target is reached
            if abs(e_x) < self.ur_target_tolerance and abs(e_y) < self.ur_target_tolerance and abs(e_z) < self.ur_target_tolerance and abs(e_phi) < self.ur_target_tolerance * 2 :
                self.ur_command.linear.x = 0
                self.ur_command.linear.y = 0
                self.ur_command.linear.z = 0
                self.ur_twist_publisher.publish(self.ur_command)
                rospy.sleep(0.1)
                break
            
            rate.sleep()
    

    
    def broadcast_target_pose(self, target_pose = Pose()):
        self.ur_target_pose_broadcaster.sendTransform((target_pose.position.x, target_pose.position.y, target_pose.position.z), (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w), rospy.Time.now(), "/ur_target_pose", self.ur_base_link_frame_id)
    
    def limit_velocity(self, ur_command, ur_command_old):
        vel_scale = 1.0
        
        # limit velocity
        if abs(ur_command.linear.x) > self.ur_velocity_limit:
            vel_scale = self.ur_velocity_limit / abs(ur_command.linear.x)
        if abs(ur_command.linear.y) > self.ur_velocity_limit and abs(ur_command.linear.y) * vel_scale > self.ur_velocity_limit:
            vel_scale = self.ur_velocity_limit / abs(ur_command.linear.y)
        if abs(ur_command.linear.z) > self.ur_velocity_limit and abs(ur_command.linear.z) * vel_scale > self.ur_velocity_limit:
            vel_scale = self.ur_velocity_limit / abs(ur_command.linear.z)
        
        # apply vel_scale
        ur_command.linear.x = ur_command.linear.x * vel_scale
        ur_command.linear.y = ur_command.linear.y * vel_scale
        ur_command.linear.z = ur_command.linear.z * vel_scale    
        
        vel_scale = 1.0
        # limit acceleration
        if abs(ur_command.linear.x - ur_command_old.linear.x) > self.ur_acceleration_limit:
            vel_scale = self.ur_acceleration_limit / abs(ur_command.linear.x - ur_command_old.linear.x)
        if abs(ur_command.linear.y - ur_command_old.linear.y) > self.ur_acceleration_limit and abs(ur_command.linear.y - ur_command_old.linear.y) * vel_scale > self.ur_acceleration_limit:
            vel_scale = self.ur_acceleration_limit / abs(ur_command.linear.y - ur_command_old.linear.y)
        if abs(ur_command.linear.z - ur_command_old.linear.z) > self.ur_acceleration_limit and abs(ur_command.linear.z - ur_command_old.linear.z) * vel_scale > self.ur_acceleration_limit:
            vel_scale = self.ur_acceleration_limit / abs(ur_command.linear.z - ur_command_old.linear.z)
            
        # apply vel_scale
        ur_command.linear.x = ur_command.linear.x * vel_scale
        ur_command.linear.y = ur_command.linear.y * vel_scale
        ur_command.linear.z = ur_command.linear.z * vel_scale
        
        # limit angular velocity
        if abs(ur_command.angular.z) > self.ur_angular_velocity_limit:
            vel_scale = self.ur_angular_velocity_limit / abs(ur_command.angular.z)
            
        # limit angular acceleration
        if abs(ur_command.angular.z - ur_command_old.angular.z) > self.ur_angular_acceleration_limit:
            vel_scale = self.ur_angular_acceleration_limit / abs(ur_command.angular.z - ur_command_old.angular.z)

        # apply vel_scale
        ur_command.angular.z = ur_command.angular.z * vel_scale
        
        return ur_command
    
    
    def ur_pose_callback(self, msg):
        self.ur_pose_current = msg.pose
    
    
if __name__ == "__main__":
    MoveURToStartPose().move_ur_to_start_pose()
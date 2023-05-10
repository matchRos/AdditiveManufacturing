#!/usr/bin/env python3

import rospy
import tf
from helper_nodes.control_ur_helper import Control_ur_helper
from tf import transformations
from std_msgs.msg import Int32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Transform
import math
from copy import deepcopy

class Control_ur():
    
    def config(self):
        # self.Kpx = -0.4
        # self.Kpy = -0.4
        # self.Kpz = 0.0
        # self.Kp_mir = 0.1  
        # self.ur_acceleration_limit = 1.0
        # self.ur_jerk_limit = 0.3
        # self.ur_velocity_limit = 0.15
        # self.ur_target_velocity = 0.1
        self.path_distance_between_points = 0.01
        self.ur_scanner_angular_offset = rospy.get_param("~ur_scanner_angular_offset", 0.995)
    
    
    def __init__(self):
        rospy.init_node("control_ur_node")
        Control_ur_helper(self)    
        self.config()
        
    
    
    def main(self):

        # get path from parameter server
        self.ur_path_array = rospy.get_param("~ur_path_array")
        self.mir_path_array = rospy.get_param("~mir_path_array")
                
        # get length factor
        self.length_factor = rospy.get_param("~length_factor")
        
        # compute the distance traveled along the path at each point
        self.compute_path_lengths()
        
        # get the transform between mir and ur
        self.mir_ur_transform = Transform()
        self.get_mir_ur_transform()
        
        # start moving the mir
        self.mir_target_velocity = Twist()
        self.mir_target_velocity.linear.x = self.ur_target_velocity * self.length_factor * 0.80
        self.mir_target_velocity_publisher.publish(self.mir_target_velocity)
        
        # wait for the subscriber to receive the first pose message
        rospy.loginfo("Waiting for mir cmd_vel message")
        rospy.wait_for_message(self.mir_cmd_vel_topic, Twist)
        rospy.loginfo("Received mir cmd_vel message")
        # wait for the mir to start moving
        
        
        # start control loop
        self.control()
        
        rospy.sleep(2)
    
    
    def control(self):
                
        self.time_old = rospy.Time.now()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():     
            # get target pose from path
            ur_target_pose_global = Pose()
            ur_target_pose_global.position.x = self.ur_path_array[self.path_index][0]
            ur_target_pose_global.position.y = self.ur_path_array[self.path_index][1]
            ur_target_pose_global.position.z = self.ur_path_array[self.path_index][2]
            ur_target_pose_global.orientation.x = self.ur_path_array[self.path_index][3]
            ur_target_pose_global.orientation.y = self.ur_path_array[self.path_index][4]
            ur_target_pose_global.orientation.z = self.ur_path_array[self.path_index][5]
            ur_target_pose_global.orientation.w = self.ur_path_array[self.path_index][6]
            
            # broadcast target pose
            self.ur_target_pose_broadcaster.sendTransform((ur_target_pose_global.position.x, ur_target_pose_global.position.y, ur_target_pose_global.position.z), (ur_target_pose_global.orientation.x, ur_target_pose_global.orientation.y, ur_target_pose_global.orientation.z, ur_target_pose_global.orientation.w), rospy.Time.now(), "ur_global_target_pose", "map")
                        
            mir_target_pose_global = Pose()
            mir_target_pose_global.position.x = self.mir_path_array[self.path_index][0]
            mir_target_pose_global.position.y = self.mir_path_array[self.path_index][1]
            
            # # broadcast mir target pose
            self.ur_target_pose_broadcaster.sendTransform((mir_target_pose_global.position.x, mir_target_pose_global.position.y, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "mir_global_target_pose", "map")
            
            # compute ur target pose in base_link frame
            x = ur_target_pose_global.position.x - self.mir_pose.position.x
            y = ur_target_pose_global.position.y - self.mir_pose.position.y
            #mir_angle = self.mir_path_array[self.path_index][2]
            mir_angle = transformations.euler_from_quaternion([self.mir_pose.orientation.x, self.mir_pose.orientation.y, self.mir_pose.orientation.z, self.mir_pose.orientation.w])[2]

            ur_target_pose_local = Pose()
            ur_target_pose_local.position.x = x * math.cos(mir_angle) + y * math.sin(mir_angle) 
            ur_target_pose_local.position.y = -x * math.sin(mir_angle) + y * math.cos(mir_angle)
            ur_target_pose_local.position.z = ur_target_pose_global.position.z 
            
            # broadcast target pose
            self.ur_target_pose_broadcaster.sendTransform((ur_target_pose_local.position.x, ur_target_pose_local.position.y, ur_target_pose_local.position.z), (ur_target_pose_global.orientation.x, ur_target_pose_global.orientation.y, ur_target_pose_global.orientation.z, ur_target_pose_global.orientation.w), rospy.Time.now(), "ur_local_target_pose", "mur620c/mir/base_link")
            
            
            # UR is mounted backwards, so we need to invert the x-axis and y-axis
            ur_target_pose_local.position.x = -ur_target_pose_local.position.x 
            ur_target_pose_local.position.y = -ur_target_pose_local.position.y
            
            # add the transform between mir and ur_base_link
            ur_target_pose_base = Pose()
            ur_target_pose_base.position.x = ur_target_pose_local.position.x + self.mir_ur_transform.translation.x
            ur_target_pose_base.position.y = ur_target_pose_local.position.y + self.mir_ur_transform.translation.y
            ur_target_pose_base.position.z = ur_target_pose_local.position.z - self.mir_ur_transform.translation.z
            ur_target_pose_base.orientation.w = 1.0

            # broadcast target pose
            self.ur_target_pose_broadcaster.sendTransform((ur_target_pose_base.position.x, ur_target_pose_base.position.y, ur_target_pose_base.position.z), (ur_target_pose_base.orientation.x, ur_target_pose_base.orientation.y, ur_target_pose_base.orientation.z, ur_target_pose_base.orientation.w), rospy.Time.now(), "target_point", self.ur_base_link_frame_id)
            
            # compute mir vel in global frame
            mir_vel_global = self.compute_mir_vel_global(self.mir_cmd_vel, mir_angle)
            
            # compute the ur_path in global frame

            ur_path_velociy_global = Twist()
            ur_path_velociy_global.linear.x = self.ur_target_velocity * math.cos(mir_angle)
            ur_path_velociy_global.linear.y = self.ur_target_velocity * math.sin(mir_angle)            
                              
            # compute current tcp angle
            ur_target_phi = transformations.euler_from_quaternion([self.ur_pose.orientation.x, self.ur_pose.orientation.y, self.ur_pose.orientation.z, self.ur_pose.orientation.w])[2]
            current_tcp_angle = transformations.euler_from_quaternion([self.ur_pose.orientation.x, self.ur_pose.orientation.y, self.ur_pose.orientation.z, self.ur_pose.orientation.w])[2]
                                    
            # compute the control law
            ur_twist_command = Twist()
            ur_twist_command.linear.x = self.Kpx * (ur_target_pose_base.position.x + self.ur_pose.position.x)
            ur_twist_command.linear.y = self.Kpy * (ur_target_pose_base.position.y + self.ur_pose.position.y)
            ur_twist_command.linear.z = self.Kpz * (ur_target_pose_base.position.z - self.ur_pose.position.z)
            
            e_phi = ur_target_phi - current_tcp_angle + self.ur_scanner_angular_offset
            if e_phi > math.pi:
                e_phi -= 2*math.pi
            elif e_phi < -math.pi:
                e_phi += 2*math.pi
                
            print("e_phi: " + str(e_phi))
            ur_twist_command.angular.z = self.Kp_phi * e_phi
                                    
            # compute path speed
            path_speed = self.compute_path_speed_and_distance(ur_path_velociy_global)
            
            # check if next path point is reached
            if self.path_distance + path_speed > self.path_lengths[self.path_index]:
                self.path_index += 1
                path_index_msg = Int32()
                path_index_msg.data = self.path_index
                self.path_index_publisher.publish(path_index_msg)          
            
            # control mir velocity
            self.control_mir_velocity(mir_target_pose_global)
                           
            # limit velocity
            ur_command = self.limit_velocity(ur_twist_command, self.ur_command_old)
                        
            # publish command
            self.ur_command_old = ur_command
            self.ur_twist_publisher.publish(ur_twist_command)
            
            rate.sleep()
           
    def compute_mir_vel_global(self, mir_vel_local = Twist(),mir_angle = 0.0):
        mir_vel_global = Twist()
        mir_vel_global.linear.x = mir_vel_local.linear.x * math.cos(mir_angle) - mir_vel_local.linear.y * math.sin(mir_angle)
        mir_vel_global.linear.y = mir_vel_local.linear.x * math.sin(mir_angle) + mir_vel_local.linear.y * math.cos(mir_angle)
        mir_vel_global.angular.z = mir_vel_local.angular.z
        return mir_vel_global
        
            
    
    def compute_path_speed_and_distance(self,ur_command):
        now = rospy.Time.now()
        dt = (now - self.time_old).to_sec()
        path_speed = math.sqrt(ur_command.linear.x**2 + ur_command.linear.y**2) 
        self.path_distance = self.path_distance + path_speed * dt
        self.time_old = now
        return path_speed * dt
       
    def get_mir_ur_transform(self):
        tf_listener = tf.TransformListener()
        # wait for transform
        tf_listener.waitForTransform(self.tf_prefix + "mir/base_link", self.tf_prefix + "UR10_r/base_link", rospy.Time(0), rospy.Duration(4.0))
        lin, ang = tf_listener.lookupTransform(self.tf_prefix + "mir/base_link", self.tf_prefix + "UR10_r/base_link", rospy.Time(0))
        
        self.mir_ur_transform.translation.x = lin[0]
        self.mir_ur_transform.translation.y = lin[1]
        self.mir_ur_transform.translation.z = lin[2]
        q = transformations.quaternion_from_euler(ang[0], ang[1], ang[2])
        self.mir_ur_transform.rotation.x = q[0]
        self.mir_ur_transform.rotation.y = q[1]
        self.mir_ur_transform.rotation.z = q[2]
        self.mir_ur_transform.rotation.w = q[3]
    
    
    
    def control_mir_velocity(self,target_pose = Pose()):
        error = math.sqrt((target_pose.position.x - self.mir_pose.position.x)**2 + (target_pose.position.y - self.mir_pose.position.y)**2)
        self.mir_target_velocity.linear.x = self.ur_target_velocity * self.length_factor * 0.80 + self.Kp_mir * error
        self.mir_target_velocity_publisher.publish(self.mir_target_velocity)
    
    def compute_path_lengths(self):
        # compute path lengths
        self.path_lengths = [0.0]
        
        for i in range(len(self.ur_path_array)-1):
            self.path_lengths.append(self.path_lengths[i] + math.sqrt((self.ur_path_array[i][0] - self.ur_path_array[i+1][0])**2 + (self.ur_path_array[i][1] - self.ur_path_array[i+1][1])**2 + (self.ur_path_array[i][2] - self.ur_path_array[i+1][2])**2))
            
    
    
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
            print("limiting acceleration")
        if abs(ur_command.linear.y - ur_command_old.linear.y) > self.ur_acceleration_limit and abs(ur_command.linear.y - ur_command_old.linear.y) * vel_scale > self.ur_acceleration_limit:
            vel_scale = self.ur_acceleration_limit / abs(ur_command.linear.y - ur_command_old.linear.y)
            print("limiting acceleration")
        if abs(ur_command.linear.z - ur_command_old.linear.z) > self.ur_acceleration_limit and abs(ur_command.linear.z - ur_command_old.linear.z) * vel_scale > self.ur_acceleration_limit:
            vel_scale = self.ur_acceleration_limit / abs(ur_command.linear.z - ur_command_old.linear.z)
            
        # apply vel_scale
        ur_command.linear.x = ur_command.linear.x * vel_scale
        ur_command.linear.y = ur_command.linear.y * vel_scale
        ur_command.linear.z = ur_command.linear.z * vel_scale

        return ur_command
    
    
    
    
    
if __name__ == "__main__":
    Control_ur().main()
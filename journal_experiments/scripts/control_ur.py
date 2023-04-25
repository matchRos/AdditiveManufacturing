#!/usr/bin/env python3

import rospy
import tf
from tf import transformations
from std_msgs.msg import Int32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Transform
import math

class Control_ur():
    
    def config(self):
        self.Kpx = -0.4
        self.Kpy = 0.4
        self.Kpz = 0.0
        self.Kpffx = 0.0  # feed-forward gain
        self.kpffy = 0.0
        self.ur_target_tolerance = 0.02
        self.ur_acceleration_limit = 0.015
        self.ur_jerk_limit = 0.3
        self.ur_velocity_limit = 0.1
        self.ur_target_velocity = 0.03
        self.path_distance_between_points = 0.01
        pass
    
    
    def __init__(self):
        rospy.init_node("control_ur_node")
        
        
        self.ur_command_topic = rospy.get_param("~ur_command_topic", "/mur620c/UR10_r/twist_controller/command_safe")
        self.ur_pose_topic = rospy.get_param("~ur_pose_topic", "/mur620c/UR10_r/ur_calibrated_pose")
        self.ur_base_link_frame_id = rospy.get_param("~ur_base_link_frame_id", "mur620c/UR10_r/base_link")
        self.mir_cmd_vel_topic = rospy.get_param("~mir_cmd_vel_topic", "/mur620c/mir/cmd_vel")
        self.tf_prefix = rospy.get_param("~tf_prefix", "mur620c/")
        self.ur_twist_publisher = rospy.Publisher(self.ur_command_topic, Twist, queue_size=1)
        self.ur_target_pose_broadcaster = tf.TransformBroadcaster()
        self.initial_run = True
        rospy.Subscriber(self.ur_pose_topic, PoseStamped, self.ur_pose_callback)
        rospy.Subscriber(self.mir_cmd_vel_topic, Twist, self.mir_cmd_vel_callback)
        
        
        self.ur_target_pose_broadcaster = tf.TransformBroadcaster()
        #rospy.Subscriber("/path_index", Int32, self.path_index_callback)
        self.path_index_publisher = rospy.Publisher('/path_index', Int32, queue_size=1)
        self.mir_target_velocity_publisher = rospy.Publisher('/mir_target_velocity', Twist, queue_size=1)
        self.ur_command = Twist()
        self.ur_command_old = Twist()
        self.path_index = 1
        self.path_speed = 0.0
        self.path_distance = 0.0
        
        self.path_index_timestamp = rospy.Time.now()
        
        
        self.config()
        
    
    
    def main(self):

        # get path from parameter server
        self.ur_path_array = rospy.get_param("~ur_path_array")
        self.mir_path_array = rospy.get_param("~mir_path_array")
        
        # get length factor
        self.length_factor = rospy.get_param("~length_factor")
        
        # get the transform between mir and ur
        self.mir_ur_transform = Transform()
        self.get_mir_ur_transform()
        
        # start moving the mir
        mir_target_velocity = Twist()
        mir_target_velocity.linear.x = self.ur_target_velocity * self.length_factor
        self.mir_target_velocity_publisher.publish(mir_target_velocity)
        
        # start control loop
        self.control()
        
        rospy.sleep(2)
    
    
    def control(self):
                
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():            
            # get target pose from path
            ur_target_pose_global = Pose()
            ur_target_pose_global.position.x = self.ur_path_array[self.path_index][0]
            ur_target_pose_global.position.y = self.ur_path_array[self.path_index][1]
            ur_target_pose_global.position.z = self.ur_path_array[self.path_index][2]
                        
            # compute ur target pose in base_link frame
            x = ur_target_pose_global.position.x - self.mir_path_array[self.path_index][0]
            y = ur_target_pose_global.position.y - self.mir_path_array[self.path_index][1]
            mir_angle = self.mir_path_array[self.path_index][2]

            ur_target_pose_local = Pose()
            ur_target_pose_local.position.x = x * math.cos(mir_angle) + y * math.sin(mir_angle) 
            ur_target_pose_local.position.y = -x * math.sin(mir_angle) + y * math.cos(mir_angle)
            ur_target_pose_local.position.z = ur_target_pose_global.position.z - self.mir_ur_transform.translation.z
            ur_target_pose_local.orientation.w = 1.0
            
            # UR is mounted backwards, so we need to invert the x-axis and y-axis
            ur_target_pose_local.position.x = -ur_target_pose_local.position.x 
            ur_target_pose_local.position.y = -ur_target_pose_local.position.y
            
            # add the transform between mir and ur_base_link
            ur_target_pose_local.position.x = ur_target_pose_local.position.x + self.mir_ur_transform.translation.x
            ur_target_pose_local.position.y = ur_target_pose_local.position.y + self.mir_ur_transform.translation.y
            
            # broadcast target pose
            self.ur_target_pose_broadcaster.sendTransform((ur_target_pose_local.position.x, ur_target_pose_local.position.y, ur_target_pose_local.position.z), (ur_target_pose_local.orientation.x, ur_target_pose_local.orientation.y, ur_target_pose_local.orientation.z, ur_target_pose_local.orientation.w), rospy.Time.now(), "target_point", self.ur_base_link_frame_id)
            
            # compute the resulting velocity of the UR base_link from the mir velocity command
            mir_ur_distance = math.sqrt(pow(self.mir_ur_transform.translation.x, 2) + pow(self.mir_ur_transform.translation.y, 2))
            ur_base_link_rot_speed = self.mir_cmd_vel.angular.z * mir_ur_distance
            ur_mount_angle = math.atan2(self.mir_ur_transform.translation.y, self.mir_ur_transform.translation.x)
            
            ur_base_link_velocity = Twist()
            ur_base_link_velocity.linear.x = self.mir_cmd_vel.linear.x + ur_base_link_rot_speed * math.sin(ur_mount_angle)
            ur_base_link_velocity.linear.y = self.mir_cmd_vel.linear.y - ur_base_link_rot_speed * math.cos(ur_mount_angle)
            
            # compute the control law
            ur_twist_command = Twist()
            ur_twist_command.linear.x = self.Kpx * (ur_target_pose_local.position.x - self.ur_pose.position.x)
            ur_twist_command.linear.y = self.Kpy * (ur_target_pose_local.position.y - self.ur_pose.position.y)
            ur_twist_command.linear.z = self.Kpz * (ur_target_pose_local.position.z - self.ur_pose.position.z)
            
            # add feed forward term
            ur_twist_command.linear.x = ur_twist_command.linear.x + ur_base_link_velocity.linear.x
            ur_twist_command.linear.y = ur_twist_command.linear.y + ur_base_link_velocity.linear.y
            
            # set timestamp on initial run and save initial target pose
            if self.initial_run:
                self.time_old = rospy.Time.now()
                self.initial_run = False
                self.integrated_ur_target_pose = ur_target_pose_local
            
            # compute time difference
            now = rospy.Time.now()
            dt = (now - self.time_old).to_sec()
            
            # update target pose based on target velocity (as a check)
            self.integrated_ur_target_pose.position.x += ur_twist_command.linear.x * rate.sleep_dur.to_sec()
            self.integrated_ur_target_pose.position.y += ur_twist_command.linear.y * rate.sleep_dur.to_sec()
            self.integrated_ur_target_pose.position.z += ur_twist_command.linear.z * rate.sleep_dur.to_sec()
            
            
            # compute target velocity in 
            continue
            
            # limit velocity
            ur_command = self.limit_velocity(self.ur_command, self.ur_command_old)
            
            # set timestamp on initial run
            if self.initial_run:
                self.time_old = rospy.Time.now()
                self.initial_run = False
            
            # compute path speed
            path_speed = self.compute_path_speed_and_distance(ur_command)
            
            # increase path index periodically based on the target velocity
            time_per_point = self.path_distance_between_points / self.ur_target_velocity
            if abs(rospy.Time.now() - self.path_index_timestamp) > rospy.Duration(time_per_point):
                self.path_index += 1
                path_index_msg = Int32()
                path_index_msg.data = self.path_index
                self.path_index_publisher.publish(path_index_msg)
                self.path_index_timestamp = rospy.Time.now() 
                continue         
            
            # self.broadcast_target_pose(target_pose)
            # check if next path point is reached
            if self.path_distance + path_speed > self.path_lengths[self.path_index]:
                self.path_index += 1
                path_index_msg = Int32()
                path_index_msg.data = self.path_index
                self.path_index_publisher.publish(path_index_msg)
                continue
            
            # publish command
            self.ur_command_old = ur_command
            self.ur_command_publisher.publish(ur_command)
            
            rate.sleep()
    
    def compute_path_speed_and_distance(self,ur_command):
        now = rospy.Time.now()
        dt = (now - self.time_old).to_sec()
        path_speed = math.sqrt(ur_command.linear.x**2 + ur_command.linear.y**2 + ur_command.linear.z**2) 
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
    
    def ur_pose_callback(self, data = PoseStamped()):
        self.ur_pose = data.pose
    
    
    def compute_path_lengths(self):
        # compute path lengths
        self.path_lengths = [0.0]
        
        for i in range(len(self.ur_path.poses)-1):
            self.path_lengths.append(self.path_lengths[i] + math.sqrt((self.ur_path.poses[i+1].pose.position.x - self.ur_path.poses[i].pose.position.x)**2 + (self.ur_path.poses[i+1].pose.position.y - self.ur_path.poses[i].pose.position.y)**2 + (self.ur_path.poses[i+1].pose.position.z - self.ur_path.poses[i].pose.position.z)**2))
    
    def mir_cmd_vel_callback(self, msg = Twist()):
        self.mir_cmd_vel = msg
    
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

        return ur_command
    
    
    
    
    
if __name__ == "__main__":
    Control_ur().main()
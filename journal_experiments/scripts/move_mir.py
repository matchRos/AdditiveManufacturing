#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose
from tf import transformations, broadcaster, listener
import math

class Move_mir():
    
    def config(self):
        self.target_pose.pose.position.x = 2.5
        self.target_pose.pose.orientation.w = 1.0
        self.start_pose.pose.orientation.w = 1.0
        self.Kp = -0.2
        self.Ktheta = 0.02
        self.angular_vel_limit = 0.1
        self.linear_vel_limit = 0.08
        self.control_rate = 100
        self.ur_vel_limit = 0.11
        self.ur_acc_limit = 0.7
    
    
    def __init__(self):
        rospy.init_node("move_mir")
        self.start_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.cmd_vel = Twist()
        self.br = broadcaster.TransformBroadcaster()
        self.listener = listener.TransformListener()
        self.config()
        self.ur_command_old = Twist()
        self.cmd_vel_pub = rospy.Publisher("/mur620b/cmd_vel", Twist, queue_size=1)
        self.ur_twist_pub = rospy.Publisher("/UR10_r/twist_controller/command", Twist, queue_size=1)
        rospy.Subscriber("/qualisys/mur620b/pose", PoseStamped, self.pose_callback)
        
        rospy.sleep(1)
        
        self.move()
        
        
    def move(self):
        rate = rospy.Rate(self.control_rate)
        
        for i in range(100):
        
            #orient toward target
            while not rospy.is_shutdown():
                curr_angle = transformations.euler_from_quaternion([self.actual_pose.orientation.x, self.actual_pose.orientation.y, self.actual_pose.orientation.z, self.actual_pose.orientation.w])[2]
                print("current angle: ", curr_angle)
                
                target_angle = math.atan2(self.target_pose.pose.position.y - self.actual_pose.position.y, self.target_pose.pose.position.x - self.actual_pose.position.x)
                print("target angle: ", target_angle)
            
                if self.target_pose.pose.position.x < 0:
                    curr_angle = curr_angle + math.pi
            
                # compute angular error
                angle_error = target_angle - curr_angle
                
                if angle_error > math.pi:
                    angle_error = angle_error - 2*math.pi
                elif angle_error < -math.pi:
                    angle_error = angle_error + 2*math.pi
            
            
                # compute angular velocity
                vel_ang = 0.3 * angle_error
                
                # limit angular velocity
                if abs(vel_ang) > self.angular_vel_limit:
                    vel_ang = self.angular_vel_limit * vel_ang / abs(vel_ang)
                    
                # set cmd_vel
                self.cmd_vel.angular.z = vel_ang 
                    
                self.cmd_vel_pub.publish(self.cmd_vel)
                
                if abs(angle_error) < 0.002:
                    break   
                
                rate.sleep()    
                    
            
        
                
            
            while not rospy.is_shutdown():
                # compute current and target angle in euler
                curr_angle = transformations.euler_from_quaternion([self.actual_pose.orientation.x, self.actual_pose.orientation.y, self.actual_pose.orientation.z, self.actual_pose.orientation.w])[2]
                target_angle = math.atan2(self.target_pose.pose.position.y - self.actual_pose.position.y, self.target_pose.pose.position.x - self.actual_pose.position.x)
                
                if curr_angle > math.pi:
                    curr_angle = curr_angle - 2*math.pi
                elif curr_angle < -math.pi:
                    curr_angle = curr_angle + 2*math.pi
                
                # compute angular error
                angle_error = target_angle - curr_angle
                #print("angle error: ", angle_error)
                
                # compute linear error
                linear_error = math.sqrt((self.target_pose.pose.position.x - self.actual_pose.position.x)**2 + (self.target_pose.pose.position.y - self.actual_pose.position.y)**2)

                # compute linear velocity
                vel_lin = self.Kp * linear_error
                
                                    
                if angle_error > 0.5 * math.pi:
                    angle_error = angle_error - math.pi
                    vel_lin = -vel_lin
                elif angle_error < -0.5 * math.pi:
                    angle_error = angle_error + math.pi
                    vel_lin = -vel_lin
                    
                if angle_error > math.pi:
                    angle_error = angle_error - 2*math.pi
                elif angle_error < -math.pi:
                    angle_error = angle_error + 2*math.pi
                
                print("angle error: ", angle_error)
                
                # compute angular velocity
                vel_ang = self.Ktheta * angle_error * 0.2
                
                # if vel_lin > 0:
                #     vel_ang = 0.0
                
                # limit angular velocity
                if abs(vel_ang) > self.angular_vel_limit:
                    vel_ang = self.angular_vel_limit * vel_ang / abs(vel_ang)
                
                
                # limit linear velocity
                if abs(vel_lin) > self.linear_vel_limit:
                    vel_lin = self.linear_vel_limit * vel_lin / abs(vel_lin)
                
                
                # set cmd_vel
                self.cmd_vel.linear.x = vel_lin
                self.cmd_vel.angular.z = vel_ang  
                
                
                
                # publish cmd_vel
                self.cmd_vel_pub.publish(self.cmd_vel)
                
                if abs(linear_error) < 0.12:
                    break
                
                
                # broadcast target pose
                
                self.br.sendTransform((self.target_pose.pose.position.x, self.target_pose.pose.position.y, self.target_pose.pose.position.z), (self.target_pose.pose.orientation.x, self.target_pose.pose.orientation.y, self.target_pose.pose.orientation.z, self.target_pose.pose.orientation.w), rospy.Time.now(), "target_pose", "mocap")
                
                ##### UR #####
                
                # get UR pose
                lin_world, ang_world = self.listener.lookupTransform("mocap", "mur620b/UR10_r/tool0", rospy.Time(0))
                lin_local, ang_local = self.listener.lookupTransform("mur620b/UR10_r/base_link", "mur620b/UR10_r/tool0", rospy.Time(0))
                                
                # compute target pose
                target_pose_ur = Pose()
                target_pose_ur.position.x = self.actual_pose.position.x + 0.5
                target_pose_ur.position.y = -1.2 + 0.3 * math.sin(1.5 * self.actual_pose.position.x)
                
                self.br.sendTransform((target_pose_ur.position.x, target_pose_ur.position.y, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "target_pose_ur", "mocap")
                
                # compute error
                e_x = target_pose_ur.position.x - lin_world[0]
                e_y = target_pose_ur.position.y - lin_world[1]
                
                # compute command
                ur_command = Twist()
                ur_command.linear.x = 0.2 * e_x 
                ur_command.linear.y = 0.5 * e_y #+ 0.1 * math.cos(1.5 * self.actual_pose.position.x)
                
                # limit linear velocity
                if abs(ur_command.linear.x) > self.ur_vel_limit:
                    ur_command.linear.x = self.ur_vel_limit * ur_command.linear.x / abs(ur_command.linear.x)
                    
                if abs(ur_command.linear.y) > self.ur_vel_limit:
                    ur_command.linear.y = self.ur_vel_limit * ur_command.linear.y / abs(ur_command.linear.y)
                
                # limit acceleration
                if abs(ur_command.linear.x - self.ur_command_old.linear.x) > self.ur_acc_limit:
                    ur_command.linear.x = self.ur_command_old.linear.x + self.ur_acc_limit * ur_command.linear.x / abs(ur_command.linear.x)
                    
                if abs(ur_command.linear.y - self.ur_command_old.linear.y) > self.ur_acc_limit:
                    ur_command.linear.y = self.ur_command_old.linear.y + self.ur_acc_limit * ur_command.linear.y / abs(ur_command.linear.y)
                    
                # publish command
                self.ur_twist_pub.publish(ur_command)
                self.ur_command_old = ur_command
                
                
                rate.sleep()
        
            self.target_pose.pose.position.x *= -1.0
        
        
    def pose_callback(self,msg = PoseStamped):
        self.actual_pose = msg.pose
        #print(self.pose)
        
        
if __name__=="__main__":
    exe = Move_mir()
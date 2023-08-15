#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Twist
import tf
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from std_msgs.msg import Int32, Int16, Float32

class Control_ur_helper():

    def __init__(self,base_node):
        self.base_node = base_node
        
        self.base_node.ur_command_topic = rospy.get_param("~ur_command_topic", "/mur620c/UR10_r/twist_controller/command_safe")
        self.base_node.ur_pose_topic = rospy.get_param("~ur_pose_topic", "/mur620c/UR10_r/ur_calibrated_pose")
        self.base_node.mir_pose_topic = rospy.get_param("~mir_pose_topic", "/mur620c/robot_pose")
        self.lateral_nozzle_pose_override_topic = rospy.get_param("~lateral_nozzle_pose_override_topic", "/lateral_nozzle_pose_override")
        self.base_node.lateral_nozzle_pose_override = 0.0
        self.base_node.ur_base_link_frame_id = rospy.get_param("~ur_base_link_frame_id", "mur620c/UR10_r/base_link")
        self.base_node.mir_cmd_vel_topic = rospy.get_param("~mir_cmd_vel_topic", "/mur620c/cmd_vel")
        self.base_node.tf_prefix = rospy.get_param("~tf_prefix", "mur620c/")
        self.base_node.ur_twist_publisher = rospy.Publisher(self.base_node.ur_command_topic, Twist, queue_size=1)
        self.base_node.ur_target_pose_broadcaster = tf.TransformBroadcaster()
        self.base_node.listener = tf.TransformListener()
        self.base_node.initial_run = True

        self.base_node.path_index_publisher = rospy.Publisher('/path_index', Int32, queue_size=1)
        self.base_node.mir_target_velocity_publisher = rospy.Publisher('/mir_target_velocity', Twist, queue_size=1)
        self.base_node.ur_command = Twist()
        self.base_node.ur_command_old = Twist()
        self.base_node.path_index = 1
        self.base_node.path_speed = 0.0
        self.base_node.path_distance = 0.0
        self.servo_position = 0
        self.servo_position_old = 0
        self.servo_position_publisher = rospy.Publisher("/servo_target_position", Int16, queue_size=1)
        
        self.base_node.path_index_timestamp = rospy.Time.now()

        # get transform from sensor frame to ur tcp
        self.base_node.listener.waitForTransform("sensor_frame", "mur620c/UR10_r/tool0", rospy.Time(0), rospy.Duration(4.0))
        self.base_node.sensor_to_tcp = self.base_node.listener.lookupTransform("sensor_frame", "mur620c/UR10_r/tool0", rospy.Time(0))

        self.setup_ddynamic_reconfigure()
        rospy.Subscriber(self.base_node.ur_pose_topic, PoseStamped, self.ur_pose_callback)
        rospy.Subscriber(self.base_node.mir_cmd_vel_topic, Twist, self.mir_cmd_vel_callback)
        rospy.Subscriber(self.base_node.mir_pose_topic, Pose, self.mir_pose_callback)
        rospy.Subscriber(self.lateral_nozzle_pose_override_topic, Float32,  self.lateral_nozzle_pose_override_callback)

    def ur_pose_callback(self, data = PoseStamped()):
        self.base_node.ur_pose = data.pose
        
    def mir_cmd_vel_callback(self, msg = Twist()):
        self.base_node.mir_cmd_vel = msg
        
    def mir_pose_callback(self, msg = Pose()):
        self.base_node.mir_pose = msg

    def dyn_rec_callback(self,config, level):
        self.base_node.ur_target_velocity = config["ur_target_velocity"]
        self.base_node.ur_velocity_limit = config["ur_velocity_limit"]
        self.base_node.ur_acceleration_limit = config["ur_acceleration_limit"]
        self.base_node.Kpx = config["Kpx"]
        self.base_node.Kpy = config["Kpy"]
        self.base_node.Kpz = config["Kpz"]
        self.base_node.Kp_phi = config["Kp_phi"]
        self.base_node.Kp_mir = config["Kp_mir"]
        self.base_node.Kp_ffx = config["Kp_ffx"]
        self.base_node.Kp_ffy = config["Kp_ffy"]
        self.base_node.Kp_lateral = config["Kp_lateral"]
        self.base_node.Kp_feedrate = config["Kp_feedrate"]
        self.servo_position = config["servo_position"]
        if self.servo_position != self.servo_position_old:
            position_msg = Int16()
            position_msg.data = self.servo_position
            self.servo_position_publisher.publish(position_msg)
        return config
    
    def setup_ddynamic_reconfigure(self):
        # Create a D(ynamic)DynamicReconfigure
        ddynrec = DDynamicReconfigure("example_dyn_rec")

        # Add variables (name, description, default value, min, max, edit_method)
        ddynrec.add_variable("ur_target_velocity", "float/double variable", 0.1, 0, 0.3)
        ddynrec.add_variable("ur_velocity_limit", "float/double variable", 0.15, 0, 0.3)
        ddynrec.add_variable("ur_acceleration_limit", "float/double variable", 0.9, 0, 2.0)
        ddynrec.add_variable("Kpx", "float/double variable", -0.7, -1.0, 1.0)
        ddynrec.add_variable("Kpy", "float/double variable", -0.7, -1.0, 1.0)
        ddynrec.add_variable("Kpz", "float/double variable", 0.50, -1.0, 1.0)
        ddynrec.add_variable("Kp_phi", "float/double variable", 0.2, -1.0, 1.0)
        ddynrec.add_variable("Kp_mir", "float/double variable", 0.1, -1.0, 1.0)
        ddynrec.add_variable("Kp_ffx", "float/double variable", 0.0, -1.0, 1.0)
        ddynrec.add_variable("Kp_ffy", "float/double variable", 0.0, -1.0, 1.0)
        ddynrec.add_variable("Kp_lateral", "float/double variable", 0.1, -1.0, 1.0)
        ddynrec.add_variable("Kp_feedrate", "float/double variable", 0.1, -1.0, 1.0)
        ddynrec.add_variable("servo_position", "integer variable", self.servo_position, 0, 1200)
        # ddynrec.add_variable("decimal", "float/double variable", -0.4, -1.0, 1.0)
        # ddynrec.add_variable("integer", "integer variable", 0, -1, 1)
        # ddynrec.add_variable("bool", "bool variable", True)
        # ddynrec.add_variable("string", "string variable", "string dynamic variable")
        # enum_method = ddynrec.enum([ ddynrec.const("Small",      "int", 0, "A small constant"),
        #                 ddynrec.const("Medium",     "int", 1, "A medium constant"),
        #                 ddynrec.const("Large",      "int", 2, "A large constant"),
        #                 ddynrec.const("ExtraLarge", "int", 3, "An extra large constant")],
        #                 "An enum example")
        # ddynrec.add_variable("enumerate", "enumerate variable", 1, 0, 3, edit_method=enum_method)

        # Start the server
        ddynrec.start(self.dyn_rec_callback)

    def lateral_nozzle_pose_override_callback(self, msg = Float32()):
        self.base_node.lateral_nozzle_pose_override = msg.data
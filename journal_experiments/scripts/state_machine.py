#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from nav_msgs.msg import Path
import roslaunch
from tf import transformations, TransformListener
import math
from copy import deepcopy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
import numpy as np

# define global variables
path = Path();
ur_path = Path();
active_robots = rospy.get_param("~active_robots", 1)
robot_names = rospy.get_param("~robot_names", ["mur620c"])
relative_positions_x = rospy.get_param("~relative_positions_x", [0])
relative_positions_y = rospy.get_param("~relative_positions_y", [0])
state = ""

# In this state the target paths for the MiR and UR are parsed
class Parse_path(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['path_parsed'])

    def execute(self, userdata):
        rospy.loginfo('Parsing path')
        
        process = launch_ros_node("create_path","journal_experiments","create_path.py")
        
        return 'path_parsed'

# In this state the MiR is moved to the first pose of the MiR path
class Move_to_start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['mir_in_start_pose'])
        
    def execute(self, userdata):
        formatin_plan_topic = rospy.get_param("/formation_plan_topic","/mir_path")
        rospy.loginfo('Waiting for mir path')
        state_publisher("idle")

        global path
        path = rospy.wait_for_message(formatin_plan_topic, Path)
        rospy.loginfo('mir path received')
        
        rospy.loginfo('Waiting for ur path')
        global ur_path
        ur_path = rospy.wait_for_message("/ur_path", Path)
        rospy.loginfo('ur path received')
        
        state_publisher("moving_to_start_pose")
        start_pose = path.poses[0].pose

        print("start_pose: ", start_pose)

        for i in range(0,active_robots):
            # compute the target pose 
            target_pose = deepcopy(start_pose)
            theta = transformations.euler_from_quaternion([target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w])[2]
            #theta = math.atan2(path.poses[1].pose.position.y - path.poses[0].pose.position.y, path.poses[1].pose.position.x - path.poses[0].pose.position.x)
            target_pose.position.x += relative_positions_x[i] * math.cos(theta) - relative_positions_y[i] * math.sin(theta)
            target_pose.position.y += relative_positions_x[i] * math.sin(theta) + relative_positions_y[i] * math.cos(theta)
            target_pose_ = [target_pose.position.x, target_pose.position.y, theta]

            # # launch the move_to_start_pose node                
            process = launch_ros_node("move_to_start_pose","journal_experiments","move_to_start_pose.py", robot_names[i] + "/", "", target_pose=target_pose_)

            # wait for the node to finish
            while process.is_alive() and not rospy.is_shutdown():
                rospy.sleep(0.1)
            rospy.loginfo(robot_names[i] + " in start pose")

        return 'mir_in_start_pose'

# In this state the UR is moved to the first pose of the UR path
class Move_UR_to_start_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ur_in_start_pose'])
        

    def execute(self, userdata):

        # Just for debugging:
        pub_ur_path_start = rospy.Publisher("/ur_path_start", PoseStamped, queue_size=1, latch=True)
        pose_start = PoseStamped()
        pose_start.header.frame_id = "map"
        pose_start.pose = ur_path.poses[1].pose
        pub_ur_path_start.publish(pose_start)
        # Just for debugging end
        
        relative_positions_x_global = path.poses[1].pose.position.x - ur_path.poses[0].pose.position.x
        relative_positions_y_global = path.poses[1].pose.position.y - ur_path.poses[0].pose.position.y
        
        mir_angle = transformations.euler_from_quaternion([path.poses[0].pose.orientation.x, path.poses[0].pose.orientation.y, path.poses[0].pose.orientation.z, path.poses[0].pose.orientation.w])[2]
        
        relative_positions_x_local = relative_positions_x_global * math.cos(mir_angle) + relative_positions_y_global * math.sin(mir_angle)
        relative_positions_y_local = -relative_positions_x_global * math.sin(mir_angle) + relative_positions_y_global * math.cos(mir_angle)

        
        # get transformation between ur and mir
        tf_listener = TransformListener()
        # wait for transform
        tf_listener.waitForTransform(robot_names[0] + "/base_link", robot_names[0] + "/UR10_l/base_link", rospy.Time(0), rospy.Duration(4.0))
        lin, ang = tf_listener.lookupTransform(robot_names[0] + "/base_link", robot_names[0] + "/UR10_l/base_link", rospy.Time(0))

        rospy.loginfo(f"UR start pose orientation: {ur_path.poses[1].pose.orientation}")

        # rotate around x so that the gripper is pointing down
        # q_rot = transformations.quaternion_from_euler(np.pi, 0, 0)
        # q_ur=transformations.quaternion_multiply(q_rot, ur_path.poses[1].pose.orientation.__reduce__()[2])
        # q_rot = transformations.quaternion_from_euler(0, 0, np.pi/2)
        # q_ur=transformations.quaternion_multiply(q_rot, q_ur)
        # q_ur = q_ur.tolist() # because param cant handle numpy types

        ur_start_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        ur_start_pose[0] = -relative_positions_x_local - lin[0]
        ur_start_pose[1] = -relative_positions_y_local - lin[1]
        ur_start_pose[2] = ur_path.poses[1].pose.position.z - lin[2]
        ur_start_pose[3] = ur_path.poses[1].pose.orientation.x
        ur_start_pose[4] = ur_path.poses[1].pose.orientation.y
        ur_start_pose[5] = ur_path.poses[1].pose.orientation.z
        ur_start_pose[6] = ur_path.poses[1].pose.orientation.w
        # ur_start_pose[3] = q_ur[0]
        # ur_start_pose[4] = q_ur[1]
        # ur_start_pose[5] = q_ur[2]
        # ur_start_pose[6] = q_ur[3]

        # TODO: ADD MIR ANGLE TO ORIENTATION? (right now done by move_ur_start_pose.py)
        
        rospy.loginfo('Executing state Move_UR_to_start_pose')
        process = launch_ros_node("move_ur_to_start_pose","journal_experiments","move_ur_to_start_pose.py", "", "", ur_start_pose=ur_start_pose, mir_angle = mir_angle)
        
        while process.is_alive() and not rospy.is_shutdown():
                rospy.sleep(0.1)
                pass
        return 'ur_in_start_pose'


# In this state the formation controller and UR controller are run
class Start_formation_controller(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['target_pose_reached', 'target_pose_not_reached'])

    def execute(self, userdata):
        state_publisher("moving_to_target_pose")

        path_array = []
        for pose in path.poses:
            theta = transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])[2]
            path_array.append([pose.pose.position.x, pose.pose.position.y , theta])

        # launch the formation_controller node
        process = launch_ros_node("control_mir","journal_experiments","control_mir.py", 
                                  "", "", path_array=path_array, active_robots=active_robots,
                                    robot_names=robot_names, relative_positions_x=relative_positions_x, relative_positions_y=relative_positions_y)

        # convert ur path to a list of poses 
        ur_path_array = []
        for pose in ur_path.poses:
            ur_path_array.append([pose.pose.position.x, pose.pose.position.y , pose.pose.position.z + 0.072, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])

        # compute length of ur path and mir path to compare them
        mir_path_length = compute_path_length(path)
        ur_path_length = compute_path_length(ur_path)
        length_factor = mir_path_length / ur_path_length

        # launch the ur controller node
        process = launch_ros_node("control_ur","journal_experiments","control_ur.py", "", "", ur_path_array=ur_path_array, length_factor=length_factor, mir_path_array = path_array)

        while process.is_alive() and not rospy.is_shutdown():
                rospy.sleep(0.1)
                pass

        state_publisher("target_pose_reached")
        return 'target_pose_reached'
        



# empty state
class Follow_trajectory(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        

    def execute(self, userdata):
        rospy.set_param("/state_machine/follow_trajectory",True)
        while not rospy.get_param("/state_machine/done") and not rospy.is_shutdown():
            rospy.sleep(0.1)
        pass
        return 'done'

def launch_ros_node(node_name, package_name, node_executable, namespace="/", node_args="", **params):
    # get param names from kwargs
    param_names = params.keys()
    # set params on param server
    for param_name in param_names:
        rospy.set_param(namespace + node_name + "/" + param_name, params[param_name])

    package = package_name
    executable = node_executable
    name = node_name
    node = roslaunch.core.Node(package=package, node_type=executable, name=name, namespace=namespace,
                                    machine_name=None, args=node_args, output="screen")
    
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)
    return process

def compute_path_length(path):
    path_length = 0.0
    for i in range(len(path.poses)-1):
        path_length += math.sqrt((path.poses[i+1].pose.position.x - path.poses[i].pose.position.x)**2 + (path.poses[i+1].pose.position.y - path.poses[i].pose.position.y)**2)
    return path_length

# main
def main():
    rospy.init_node('smach_example_state_machine')
    init_ros_param_server()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Parse_path', Parse_path(), 
                               transitions={'path_parsed':'Move_to_start'})
        smach.StateMachine.add('Move_to_start', Move_to_start(), 
                               transitions={'mir_in_start_pose':'Move_UR_to_start_pose'})
        smach.StateMachine.add('Start_formation_controller', Start_formation_controller(), 
                               transitions={'target_pose_reached':'Move_to_start','target_pose_not_reached':'Parse_path'})
        smach.StateMachine.add('Move_UR_to_start_pose', Move_UR_to_start_pose(),
                                 transitions={'ur_in_start_pose':'Start_formation_controller'})
        smach.StateMachine.add('Follow_trajectory', Follow_trajectory(),
                                transitions={'done':'outcome5'})
    # Execute SMACH plan

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

def state_publisher(state):
    state_pub = rospy.Publisher('/state_machine/state', String, queue_size=10)
    state_string = String()
    state_string.data = state
    state_pub.publish(state_string)


def init_ros_param_server():
    rospy.set_param("/state_machine/move_mir_to_start_pose",False)
    rospy.set_param("/state_machine/move_ur_to_start_pose",False)
    rospy.set_param("/state_machine/paths_received",False)
    rospy.set_param("/state_machine/trajectories_received",False)
    rospy.set_param("/state_machine/ur_trajectory_received",False)
    rospy.set_param("/state_machine/mir_trajectory_received",False)
    rospy.set_param("/state_machine/mir_initialized",False)
    rospy.set_param("/state_machine/ur_initialized",False)
    rospy.set_param("/state_machine/follow_trajectory",False)
    rospy.set_param("/state_machine/done",False)




if __name__ == '__main__':
    main()
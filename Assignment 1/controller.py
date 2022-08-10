#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot

import math
import geometry_msgs

# Use to transform between frames
tf_buffer = None
listener = None

goal_client = None      # exploration simple action client
control_client = None   # collision avoidance service client

pub = None                     #  velocitypublisher
robot_frame_id = "base_link"    

max_linear_velocity = 0.5    # Max linear velocity (m/s)
max_angular_velocity = 2.5    # Max angular velocity (rad/s)


def move(path):
    global control_client, robot_frame_id, pub

    # Call service client if the returned path is not empty and do stuff agian (collision avoidance)
    while path.poses:
        rospy.wait_for_service('get_setpoint')
        control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)

        response = control_client(path)
        setpoint = response.setpoint
        path = response.new_path

        # Transform Setpoint from service client
        transform = tf_buffer.lookup_transform(robot_frame_id, setpoint.header.frame_id, rospy.Time())
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)

        #Create Twist message from the transformed Setpoint
        msg = Twist()
        msg.angular.z = max_angular_velocity * math.atan2(transformed_setpoint.point.y, transformed_setpoint.point.x) 
        msg.linear.x = max_linear_velocity * math.hypot(transformed_setpoint.point.y, transformed_setpoint.point.x)    

        if msg.angular.z > max_angular_velocity:
            msg.angular.z = max_angular_velocity
        if msg.linear.x > max_linear_velocity:
            msg.linear.x = max_linear_velocity
        #Publish Twist
        pub.publish(msg)
        rate = rospy.Rate(10)
        rate.sleep()

    # Send 0 control Twist to stop robot
    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)

    # Get new path from action server
    get_path()



def get_path():
    global goal_client

    # Get path from action server (explore node)
    goal_client.wait_for_server()
    goal_client.send_goal(None)
    goal_client.wait_for_result()

    result = goal_client.get_result()
    path = result.path

    # Call move with path from action server
    if path.poses:
        move(path)



if __name__ == "__main__":
    # Init node
    rospy.init_node('controller')

    # Init publisher
    pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

    # Init simple action client (explorer)
    goal_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)

    # Init service client (collision avoidance)
    control_client = rospy.ServiceProxy('get_setpoint', irob_assignment_1.srv.GetSetpoint)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)         # Start receiving tf2 transformations 
    # Call get path
    get_path()

    # Spin
    rospy.spin()

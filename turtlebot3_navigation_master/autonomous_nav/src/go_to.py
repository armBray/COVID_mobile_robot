#!/usr/bin/env python

import math

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

def movebase_client(x_goal, y_goal, angle_goal):
    """
    x,y goal wrt to robot, angle in DEG
    """

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x_goal
    goal.target_pose.pose.position.y = y_goal

    # Goal angle - in DEG - converted to quaternion
    goal.target_pose.pose.orientation = Quaternion(*(quaternion_from_euler(0, 0, angle_goal*math.pi/180)))

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        # Send goal x,y,deg
        result = movebase_client(-1,-0.2,30)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

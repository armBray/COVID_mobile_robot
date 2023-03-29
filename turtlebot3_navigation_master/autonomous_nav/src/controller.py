#! /usr/bin/env python

# ROS imports
import rospy
import sys
import numpy as np
import actionlib
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from actionlib_msgs.msg import GoalID, GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionFeedback, MoveBaseGoal
goal = [0,0]
pose = [0,0]
time_goal = 0
msg = GoalStatusArray()
status_list_msg = GoalStatus()
status = 3
succed_pub = None

#attiva controller solo se abbastanza vicino al goal (entro 1 m per esempio)

def clbk_goal(data):
    print "---------I'm ALIVE------------"
    goal[0] = data.pose.position.x/map_properties.resolution
    goal[1] = data.pose.position.y/map_properties.resolution
    goal[0] = int(goal[0]) + 200
    goal[1] = int(goal[1]) + 200

def clbk_status(data):
    msg = data
    status_list_msg = data.status_list
    status_list_msg.status = 3
    msg.status_list = status_list_msg
    print msg


def main():
    global goal, msg, status, status_list_msg, succed_pub, pose, goal_distance, map_properties, time_goal
    map_properties =rospy.wait_for_message("map_metadata", MapMetaData) 
    succed_pub = rospy.Publisher("/move_base/status", GoalStatusArray, queue_size=1)
    status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, clbk_status)
    goal_sub = rospy.Subscriber("/move_base/current_goal", PoseStamped, clbk_goal)



    while not rospy.is_shutdown():

	#goal = MoveBaseGoal()
        #goal = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y]      
        print "Goal:     ", goal
	pose = rospy.wait_for_message('move_base/feedback',MoveBaseActionFeedback)
        pose = [pose.feedback.base_position.pose.position.x, 					pose.feedback.base_position.pose.position.y]   
        pose[0] = (pose[0])/map_properties.resolution
        pose[1] = (pose[1])/map_properties.resolution
        pose[0] = int(pose[0]) + 200
        pose[1] = int(pose[1]) + 200 


        pose_point = np.array((pose[0], pose[1]))
        goal_point = np.array((goal[0], goal[1]))
        goal_distance = np.linalg.norm(pose_point - goal_point)

        print "Position: ", pose
        print "Goal:     ", goal
        print "Distance: ", goal_distance
        print ""
    
        if goal_distance <= 5:
            print msg
            print "---Goal approximately reached"
            succed_pub.publish(msg)


        



    
if __name__ == '__main__':
    rospy.init_node('Control')
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controll finished.")


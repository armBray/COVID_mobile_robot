#! /usr/bin/env python

# ROS imports
import rospy
import sys
import numpy as np
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from actionlib_msgs.msg import GoalID, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionFeedback
goal = [0,0]
pose = [0,0]
time_goal = 0
msg = GoalID()
cancel_pub = None
cancel_time = 5
cancel_msg = GoalID()
#attiva controller solo se abbastanza vicino al goal (entro 1 m per esempio)
def clbk_map(data):

    pose = rospy.wait_for_message('move_base/feedback',MoveBaseActionFeedback)
    pose = [pose.feedback.base_position.pose.position.x, 					pose.feedback.base_position.pose.position.y]   
    pose[0] = (pose[0])/map_properties.resolution
    pose[1] = (pose[1])/map_properties.resolution
    pose[0] = int(pose[0]) + 200
    pose[1] = int(pose[1]) + 200 


    pose_point = np.array((pose[0], pose[1]))
    goal_point = np.array((goal[0], goal[1]))
    goal_distance = np.linalg.norm(pose_point - goal_point)

    elapsed_time = rospy.get_time() - time_goal
    #print "Map received"
    print "Position: ", pose
    print "Goal:     ", goal
    print "Distance: ", goal_distance
    print "Time:     ", elapsed_time
    print ""
    
    global cancel_pub
    data = list(data.data)
    for i in range(-5,6 ):
        for j in range(-5,6):
            if data[(goal[1]+i)*384 + goal[0]+ j]>0:
                print "---Goal cancelled"
                cancel_pub.publish(msg)
    if goal_distance <= 3:
        print "---Goal approximately reached"
        cancel_pub.publish(msg)
    if elapsed_time >= 20:
	if (goal_distance <= 10):
            print "---Goal Cancelled: time elapsed"
            cancel_pub.publish(msg)
        elif (elapsed_time >= 70):
	    if (goal_distance <= 30):
                print "---Goal Cancelled: time elapsed"
                cancel_pub.publish(msg)
            elif (elapsed_time >= 100):
                print "---Goal Cancelled: time elapsed"
            cancel_pub.publish(msg)



def main():
    global goal, msg, cancel_pub, pose, goal_distance, map_properties, time_goal
    map_properties =rospy.wait_for_message("map_metadata", MapMetaData)

    cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)

    map_sub = rospy.Subscriber('map', OccupancyGrid, clbk_map)


    while not rospy.is_shutdown():

        goal = rospy.wait_for_message('move_base/goal',MoveBaseActionGoal)
        goal = [goal.goal.target_pose.pose.position.x, goal.goal.target_pose.pose.position.y]      
	goal[0] = (goal[0])/map_properties.resolution
        goal[1] = (goal[1])/map_properties.resolution
        goal[0] = int(goal[0]) + 200
        goal[1] = int(goal[1]) + 200

        print "-> New Goal: ", goal
        
        time_goal = rospy.get_time()
        print "Time: ", time_goal


    
if __name__ == '__main__':
    rospy.init_node('Controller')
    main()

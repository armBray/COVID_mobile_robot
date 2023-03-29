#! /usr/bin/env python

import rospy
import math
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty
import matplotlib.pyplot as plt

map_properties = 0
energy_map = np.zeros((384,384),np.float64)

pose = [0,0]

def get_pose():
    init_time = rospy.get_time()
    pose = rospy.wait_for_message('move_base/feedback',MoveBaseActionFeedback)
    delta_time = rospy.get_time() - init_time

    pose = [pose.feedback.base_position.pose.position.x, pose.feedback.base_position.pose.position.y]   
    pose[0] = (pose[0])/map_properties.resolution
    pose[1] = (pose[1])/map_properties.resolution
    pose[0] = int(pose[0]) + 200
    pose[1] = int(pose[1]) + 200 

    return pose, delta_time

def get_energy_map():
        
    data = rospy.wait_for_message("/map",OccupancyGrid)
    data = list(data.data)
    for j in range(map_properties.height):
        for i in range(map_properties.width):
            energy_map[j][i] = data[j*map_properties.height + i]
    for j in range(map_properties.height):
        for i in range(map_properties.width):
            if energy_map[j][i] <> 0:
		 energy_map[j][i] = occupancy_val
def main():
    global map_properties, energy_map, occupancy_val
    occupancy_val = -15
    map_total_energy = 0
    P_l = 0.1			#instantaneous power density [mW-m2]
    res = 0.05			#grid resolution 1 = 1mt (0.05 means that the grid was divided into 20x20 piece)
    P = P_l*pow(0.2,2)		#cell instantaneous power density [mW-m2] (grid side lenght = 0.2m)
    energy_radius = 20		#spreading radious 40=1mt (N.B. 20pt in the map = 1mt -> en_rad x res = 20 = 1mt)
    map_properties =rospy.wait_for_message("map_metadata", MapMetaData)

    get_energy_map()
    print energy_map

    energy_local_map = np.zeros((energy_radius,energy_radius),np.float64)

    while not rospy.is_shutdown():
        pose, delta_time = get_pose()
        time_before_publish = rospy.get_time()
        #rospy.wait_for_message('move_base/feedback',MoveBaseActionFeedback)
	time_before_computation = rospy.get_time()
       
	#cell_energy = 0
	cell_total_energy = 0
	for i in range(-energy_radius, energy_radius):
	    for j in range(-energy_radius, energy_radius):
		cell_energy = 0
		if energy_map[j+pose[1],i+pose[0]] <> occupancy_val:
	            delta_x = float(i*res)
                    delta_y = float(j*res)
	            if not (delta_x <= 0.1 and delta_x >= -0.1 and delta_y <= 0.1 and delta_y >= -0.1):
	                #under the turtlebot energy is zero
                        cell_energy = P*delta_time/(pow(delta_x,2)+pow(delta_y,2))
		cell_total_energy += cell_energy
                energy_map[j+pose[1],i+pose[0]] += cell_energy*200 #200 is for the map visu factor
		
	map_total_energy += cell_total_energy
        #print "Compute Time: ", rospy.get_time() - time_before_computation
	#print "Publish Time: ", time_before_computation - time_before_publish
	print "Position:     ", pose
	print "Cell Energy:  ", cell_energy, " mJ"
        print "Sub-Grid En.: ", cell_total_energy, " mJ"
        print "Time spent:   ", delta_time, " s"
        print "--------------"
        print "Total Energy: ", map_total_energy, " mJ"
        print "Total Dens. : ", map_total_energy/9/10, " mJ" #9mq x 10cells #20mq x 25cells

if __name__ == '__main__':
    rospy.init_node('energy_recorder')
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Energy Mapping finished.")
        np.savetxt("src/turtlebot3_navigation_master/autonomous_nav/src/energy_map.csv", energy_map,delimiter=",")
    	plt.imshow(energy_map, cmap = 'plasma')
   	plt.show()
 

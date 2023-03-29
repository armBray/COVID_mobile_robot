#! /usr/bin/env python

# ROS imports


import rospy
import sys
from sensor_msgs.msg import LaserScan
from obstacle_detection.msg import Obstacle
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from tf.transformations import euler_from_quaternion
import numpy as np
import cv2
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
#global variable definition
to_check = np.zeros((384*384,1),np.int8)
map_properties = 0
map = np.zeros((384,384),np.int8)
map_frontier = np.zeros((384,384),np.uint8)
frontier = np.zeros((1000,1000,2),np.uint16)
number_frontier = 0
count_frontier = np.zeros((1000,1), np.uint16)
centroids = np.zeros((50,2),np.uint16)
x_0 = 140
y_0 = 220
merged = 0
pose_feedback = [0,0]
goal_done = np.zeros((2,1), np.int16)
goal = np.zeros((2,1), np.int16)
goal_abs=[0,0]
starting_point = [-3,1]
obstacle = False
laser = 0
heading = 0
xx = 0
yy = 0
limit = 3
frontier_array = np.zeros((5,1), np.uint16)
goal_array = np.zeros((20,2), np.uint16)
goal_count = 0
prices = np.zeros((50,1), np.int16)
inadmissible = [[0,0]]
count_inadmissible = 0
goal_numbers = 0

def active_cb():
    rospy.loginfo("Goal pose is now being processed by the Action Server...")


def feedback_cb(feedback):
    global pose_feedback, obstacle, goal, laser, heading, obstacle


    #To print current pose at each feedback:
    #rospy.loginfo("Feedback for goal: "+str(feedback))
    #rospy.loginfo("Feedback for goal pose received")
    pose_feedback = [feedback.base_position.pose.position.x, feedback.base_position.pose.position.y]
    orientation = [feedback.base_position.pose.orientation.x, feedback.base_position.pose.orientation.y, feedback.base_position.pose.orientation.z, feedback.base_position.pose.orientation.w]
    (roll, pitch, heading) = euler_from_quaternion (orientation)





def done_cb(status, result):
    global obstacle
    print obstacle
    if obstacle:
        rospy.loginfo("The robot is too close an obstacle -> cancel")
        obstacle = False
        return
# Terminal Status from Reference http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
    if status == 2:
        rospy.loginfo("Goal pose received a cancel request after it started executing, completed execution!")

    if status == 3:
        rospy.loginfo("Goal pose reached")

        # Per spegnerlo quando arriva
        if 0:
            rospy.loginfo("Final goal pose reached!")
            rospy.signal_shutdown("Final goal pose reached!")
        return

    if status == 4:
        rospy.loginfo("Goal pose was aborted by the Action Server")
        # Shut down if refused
        # Possiamo fargli fare qualunque cosa
        #rospy.signal_shutdown("Goal pose aborted, shutting down!")
        return

    if status == 5:
        rospy.loginfo("Goal pose has been rejected by the Action Server")
        # Idem com sopra
        #rospy.signal_shutdown("Goal pose rejected, shutting down!")
        return

    if status == 8:
        rospy.loginfo("Goal pose received a cancel request before it started executing, successfully cancelled!")


def map_update(data):
    global map, map_frontier

    print "Starting updating map"
    for j in range(map_properties.height):
        for i in range(map_properties.width):
            if data[j*map_properties.height + i]== 0:
                for k in range(-1,2):
                    for h in range(-1,2):
                        if data[(j+k)*map_properties.height + (i  + h)] == -1:
                                data[(j+k)*map_properties.height + (i + h)] = 255
    print "Map updated-> second phase"
    for j in range(map_properties.height):
        for i in range(map_properties.width):
            if data[j*map_properties.height + i] == -1:
                data[j*map_properties.height + i] = 0
            elif (data[j*map_properties.height + i]>=1 and data[j*map_properties.height + i] <=100):
                data[j*map_properties.height + i] = 0
    for j in range(map_properties.height):
        for i in range(map_properties.width):
            map_frontier[j][i] = data[j*map_properties.height + i]

    plt.imshow(map_frontier, cmap = 'gray')
    plt.show()
    print "Finished Converting"

def map_update_v2(data):
    global map, map_frontier

    print "Starting updating map"
    for j in range(map_properties.height):
        for i in range(map_properties.width):
            if data[j*map_properties.height + i]== 0:
                for k in range(-1,2):
                    for h in range(-1,2):
                        if data[(j+k)*map_properties.height + (i  + h)] == -1:
                                if obstacle_check(data, j+ k, i+h):
                                    data[(j+k)*map_properties.height + (i + h)] = 255
    print "Map updated-> second phase"
    for j in range(map_properties.height):
        for i in range(map_properties.width):
            if data[j*map_properties.height + i] == -1:
                data[j*map_properties.height + i] = 0
            elif (data[j*map_properties.height + i]>=1 and data[j*map_properties.height + i] <=100):
                data[j*map_properties.height + i] = 0
    for j in range(map_properties.height):
        for i in range(map_properties.width):
            map_frontier[j][i] = data[j*map_properties.height + i]

    #plt.imshow(map_frontier, cmap = 'gray')
    #plt.show()
    print "Finished Converting"

def neighbour(x,y):
    flag = False
    father = [x,y]


    second_father = [x,y]

    #first Check
    for i in range(-5,0):
        if map_frontier[x+i][y] == 255:
            father = [x+i,y]
            break
    #second check
    if father[0] == x and father[1] == y:
        for i in range(-5,6):
            for j in range(1,6):
                if map_frontier[x+i][y-j] == 255:
                    father = [x+i,y-j]
                    break
            if not (father[0] == x and father[1] == y):
                break
    #check for other frontiers
    for i in range(-5,0):
        for  j in range(-5,0):
            if map_frontier[x-i][y+j] == 255:
                second_father = [x-i,y+j]
                break
        if not (second_father[0] == x and second_father[1] == y):
            break

    if father[0] == x and father[1] == y:
        flag = False
    else:
        flag = True
    if second_father[0] == x and second_father[1] == y:
        merge = False
    else:
        merge = True
    return merge,flag, father, second_father

def find_father_frontier(x, y):

    for i in range(number_frontier):
        for j in range(int(count_frontier[i])):
            if x == frontier[i][j][0] and y == frontier[i][j][1]  and map_frontier[x][y] == 255:
                #print "Father found -> Frontier number: ", i
                return i
    #print "Father not found"
    return 1000

def merging_frontier(x_1, y_1, x_2, y_2):

    global number_frontier, count_frontier, frontier, merged
    numb_1 = 0
    numb_2 = 0

    numb_1 = find_father_frontier(x_1, y_1)
    numb_2 = find_father_frontier(x_2, y_2)
    #print "Found the two frontiers to merge: ", numb_1, " and ", numb_2
    if numb_1 != 1000 and numb_2!= 1000:
        if numb_1 > numb_2:
            temp = numb_1
            numb_1 = numb_2
            numb_2 = temp
    if numb_1!=numb_2 and (numb_1 != 1000 and numb_2!= 1000):
        merged = merged + 1
        for s in range(int(count_frontier[numb_2])):
            frontier[numb_1][count_frontier[numb_1]] = frontier[numb_2][s]
            count_frontier[numb_1] = count_frontier[numb_1] + 1
        print "Merge completed between frontier ", numb_1, " and ", numb_2
def frontier_computation():
    global frontier, count_frontier, number_frontier
    father = [0, 0]
    second_father = [0,0]
    father_frontier = 0
    second_father_frontier = 0
    for i in range(map_properties.height):
        for j in range(map_properties.width):
            if map_frontier[i][j] == 255:
                merge, flag, father, second_father = neighbour(i,j)
                if flag == True:
                    father_frontier = find_father_frontier(father[0], father[1])
                    if father_frontier != 1000:
                        frontier[father_frontier][count_frontier[father_frontier]]=  [i,j]
                        count_frontier[father_frontier] = count_frontier[father_frontier] + 1
                if merge == True:
                    second_father_frontier = find_father_frontier(second_father[0], second_father[1])
                    if second_father_frontier != father_frontier:
                        merging_frontier(father[0], father[1], second_father[0], second_father[1])
                if flag == False:
                    #print "New frontier found"
                    frontier[number_frontier][count_frontier] = [i, j]
                    count_frontier[number_frontier] = 1
                    number_frontier = number_frontier + 1

def obstacle_check(data,x,y):
    global to_check
    print "Checking if the cell is far enough from obstacles"
    flag = True
    for i in range(-5,6):
        for j in range(-5,6):
            #print data[(x + i)*map_properties.height + y + j]
            if data[(x + i)*map_properties.height + y + j] > 50:
                flag = False
                break
    return flag

def middle_point_computation(frontier_number):
    global frontier, count_frontier
    middle_point = np.zeros((2),np.uint16)
    print "Frontier number: ", frontier_number
    middle = int(count_frontier[frontier_number]/2)
    middle_point[0] = frontier[frontier_number][middle][0]
    middle_point[1] = frontier[frontier_number][middle][1]
    return middle_point
def centroid_computation(frontier_number):
    global frontier, count_frontier
    centroid = np.zeros((2),np.uint16)
    print "Frontier number: ", frontier_number
    sum_x = 0
    sum_y = 0
    #print frontier[frontier_number]
    for i in range(count_frontier[frontier_number]):
        sum_x = sum_x + frontier[frontier_number][i][0]
        sum_y = sum_y + frontier[frontier_number][i][1]
        #print sum_x, sum_y
        #print frontier[number_frontier][i][0]
    centroid[0] = int(sum_x/count_frontier[frontier_number])
    centroid[1] = int(sum_y/count_frontier[frontier_number])
    print centroid
    return centroid
def distance(x1,y1,x2,y2):
    x1 = int(x1)
    x2 = int(x2)
    y1 = int(y1)
    y2 = int(y2)
    dist = np.zeros(1, np.float64)
    dist = int(math.sqrt((int(int(x2-x1)**2) + int(int(y2-y1)**2))))
    return dist

def goal_computation(x,y):
    global goal, frontier_array,   goal_array, goal_count

    max = 5
    index = 0
    size = 0

    for k in range(number_frontier):
        print "Frontier number ", k, " with size ",count_frontier[k]
        if count_frontier[k] > max:
            index = k
            max = int(count_frontier[k])
    frontier_goal(index, x, y)

def frontier_goal(index,x,y):
    global goal
    distance_max = 0
    count = 0
    target = centroid_computation(index)
    print "First attempt -> centroid"
    for j in range(len(inadmissible)):
        #print target
        #print inadmissible[j]
        #print distance(inadmissible[j][0], inadmissible[j][1], target[0], target[1])
        if distance(inadmissible[j][0], inadmissible[j][1], target[0], target[1]) > 5:
                count = count + 1
    #print count, len(inadmissible)
    if count == len(inadmissible):
        goal = target
        print "Goal for the frontier number ",index," is ", target

    else:
        print "Second attempt -> closest point"
        for i in range(int(count_frontier[index])):
            target = frontier[index][i]
            count = 0
            for j in range(len(inadmissible)):
                if distance(inadmissible[j][0], inadmissible[j][1], target[0], target[1]) > 5:
                    count = count + 1
            if count == len(inadmissible):
                dist = distance(x,y, target[0], target[1])
                if dist > distance_max:
                    distance_max= dist
                    temp = target
        if distance_max != 0:
            goal = temp

def next_goal(x,y):

    distance_min = 1000
    for i in range(goal_number):
        count = 0
        for j in range(len(goal_done)):
            print goal_array[i]
            print goal_done[j]
            if  distance(goal_array[i][0], goal_array[i][1],goal_done[j][0], goal_done[j][1]) > 2:
                count = count + 1
        print count, len(goal_done)
        if count == len(goal_done):
            dist = distance(x,y,goal_array[i][0], goal_array[i][1])
            if dist <= distance_min:
                distance_min = dist
                prox_goal = goal_array[i]
                print prox_goal

    if distance_min <= 200:
        return prox_goal
    else:
        return [0,0]

def movebase_client(x_goal, y_goal, angle_goal):
    """
    x,y goal wrt to robot, angle in DEG
    """
    print "Move__base"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    print "Goal:",x_goal, y_goal
    goal.target_pose.pose.position.x = x_goal
    goal.target_pose.pose.position.y = y_goal

    # Goal angle - in DEG - converted to quaternion
    goal.target_pose.pose.orientation = Quaternion(*(quaternion_from_euler(0, 0, angle_goal*math.pi/180)))
    print "Sending goals"

    client.send_goal(goal, done_cb, active_cb, feedback_cb)

    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
def distance_computation():
    middle_points = np.zeros((60,2), np.uint16)
    to_merge = np.zeros((60,60), np.int8)
    count = np.zeros((60,1), np.uint16)
    for  i in range(number_frontier):
        middle_points[i] = middle_point_computation(i)
    print "All the points are computed"
    for i in range(number_frontier):
        for j in range(number_frontier):
            if j > i:
                print middle_points[i]
                print middle_points[j]
                if distance(middle_points[i][0], middle_points[i][1], middle_points[j][0], middle_points[j][1]) < 30:
                    print i, count[i]
                    to_merge[i][count[i]]= j
                    count[i] = count[i] + 1
                    print "Frontier ", i, " and frontier ", j, " need to be merged"
    print count
    print to_merge
    for i in range(number_frontier + 1):
        for j in range(count[number_frontier - i - 1]):
            print i, " -> ", count[number_frontier - i - 1]
            x = number_frontier - 1  - i
            print "X = ",x
            print to_merge[x][j]
            if to_merge[x][j] != 0:
                for s in range(int(count_frontier[to_merge[x][j]])):
                    frontier[x][count_frontier[x]] = frontier[to_merge[x][j]][s]
                    count_frontier[x] = count_frontier[x] + 1
                count_frontier[to_merge[x][j]] = 0

def main():
    global to_check, centroids, inadmissible, count_inadmissible
    global goal, x_0, y_0, frontier, number_frontier, count_frontier, map_properties, map, map_frontier,pose_feedback, goal
    global starting_point, goal_abs, goal_array, goal_number, goal_done
    map_properties =rospy.wait_for_message("map_metadata", MapMetaData)
    current_goal = np.zeros((2,1), np.float32)

    x_start = 0
    y_start = 0
    yaw = 0

    pose_feedback = [-3, 1]



    x_start = starting_point[0] - map_properties.origin.position.x
    y_start = starting_point[1] - map_properties.origin.position.y

    x0_cell = int(x_start/map_properties.resolution)
    y0_cell = int(y_start/map_properties.resolution)
    xx = x0_cell
    yy = y0_cell
    x_start = xx
    y_start = yy
    s = 0
    while not rospy.is_shutdown():
        goal = [0,0]
        map = np.zeros((384,384),np.int8)
        map_frontier = np.zeros((384,384),np.uint8)
        number_frontier = 0
        count_frontier = np.zeros((5000,1),np.uint8)
        frontier = np.zeros((5000,1000,2),np.uint16)
        number_frontier = 0
        merged = 0
        centroids = np.zeros((50,2),np.uint16)
        goal_array = np.zeros((20,2), np.uint16)
        goal_number = 0
        result = False
        data = rospy.wait_for_message("/map",OccupancyGrid)
        to_check = list(data.data)
        goal_done = np.zeros((1,2), np.int16)
        data = list(data.data)

        map_update_v2(data)
        map_frontier = map_frontier.T
        print "Frontier map created"

        frontier_computation()
        distance_computation()
        print "current_position: ", xx , yy

        goal_computation(x_start,y_start)
        #goal = next_goal(x_start, y_start)
        print "Goal ", goal
        if goal[0]!= 0 and goal[1]!=0:
            fine = True
        else:
            fine = False


        s = s + 1
        print "Iteration number ", s
        print "value: ", data[int(goal[1]*map_properties.height + goal[0])]
        #if data[goal[1]*map_properties.height + goal[0]] == 0:
            #print "Goal skipped, already mapped"
            #continue
        temp0 = goal[0]
        temp1 = goal[1]
        current_goal[0]= int(temp0) - 200
        current_goal[1]= int(temp1) - 200
        print goal
        print "Relative cells to the goal ", goal
        current_goal[0] = current_goal[0]*map_properties.resolution
        current_goal[1] = current_goal[1]*map_properties.resolution
        result = movebase_client(current_goal[0],current_goal[1],0)
        print "result ", result



        [xx, yy] = pose_feedback
        x_start = xx/map_properties.resolution
        y_start = yy/map_properties.resolution


        if current_goal[0] - 0.2 < xx < current_goal[0] + 0.2 and current_goal[1] - 0.2 < yy < current_goal[1] + 0.2:
            rospy.loginfo("Goal execution done!")
            #goal_done.append([goal[0],goal[1]])
        else:
            inadmissible.append([goal[0],goal[1]])
            print "inadmissible goal -> stored in the memory"
            print inadmissible

        print "The next goal is ", goal

        x_start = int(x_start) + 200
        y_start = int(y_start) + 200

if __name__ == '__main__':
    rospy.init_node('Pioneer')
    main()

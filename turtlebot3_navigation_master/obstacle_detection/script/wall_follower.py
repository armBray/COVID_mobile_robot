#!/usr/bin/env python

# Driving always straight

#settare meglio gradi destra da osservare per avere muro
import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from obstacle_detection.msg import Obstacle
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Util imports
import random
import math
import time
from numpy import sign

dist_min_right = 0
PI = 3.1415926535897
angular_speed = 0.3
linear_speed = 0.2
delta_t = 0
t0 = 0

# booleans for changing state
wall_found = 0
wall_reached = 0
min_index = 0
min_index_right = 0
min_cells = 50
ref_d = 0.5
# max distance at which our robot can see obstacles
d = 2
#amount of laser point for defining a Wall
laser_points = 100
ob = Obstacle()
Init = True
current_angle = 0
hz = 20                     # Cycle Frequency
starting_detection = True
inf = 3                     # Limit to Laser sensor range in meters, all distances above this value are
goal_direction = 0          # considered out of sensor range
wall_dist = 0.5             # Distance desired from the wall
max_speed = 0.3             # Maximum speed of the robot on meters/seconds

direction = 1               # 1 for wall on the left side of the robot (-1 for the right side)
                            # Diference between current wall measurements and previous one
angle_min = 0               # Angle, at which was measured the shortest distance between the robot and a wall
dist_front = 0              # Measured front distance

dist_min = 0                # Minimum measured distance
start = 1
starting_rotation = 0
outside = False

# Time when the last outer corner; direction and inner corner were detected or changed.
# time.time() returns the time passed since epoch
right_bool = 0 #=1 if there is some wall on the right
rotating = 0
msg = Twist()
msg.angular.z = 0
vel_pub = None

# Sensor regions
regions_ = {
        'bright': 0,
        'right': 0,
        'fright': 0,
        'front': 0,
        'left': 0,
        'bleft': 0,
        'fleft':0
}
regions_max = {
        'bright': 0,
        'right': 0,
        'fright': 0,
        'front': 0,
        'left': 0,
        'bleft': 0,
        'fleft':0
}
reached = False
oriented = 0
rate = 0


#Robot state machines
state_ = 0
state = 0
state_dict_ = {
    0: 'finding wall',
    1: 'approaching to wall',
    2: 'wall following',
    3: 'inner corner - rotation',
    4: 'outer corner - rotation',
    5: 'Outside'
}

right = 0


back = 0
bool_inner_corner = 0
bool_outer_corner = 0
outside = False

def outside_check(laser):
    outside_bool = False

    if laser.ranges[270] > 3 and laser.ranges[90]>3 and min(laser.ranges[0:45])  > 3:
        return True
    else:
        return False

def is_outer_corner(min_index,laser):

    global bool_outer_corner, regions_, wall_dist
    regions = regions_
    back = min(laser.ranges[165:194])
    right_min = min(laser.ranges[255:285])
    right_max = max(laser.ranges[255:285])
    back_right = min(laser.ranges[225:254])
    front_right = min(laser.ranges[285:320])
    corner = 180
    corner_dist = inf
    projected_corner_dist = 0
    minimum = 180
    bool_outer_corner = 0
    verti = 10
    print "right min: ",right_min, " max: ", right_max
    for i in range(180,260):
        if laser.ranges[i] < laser.ranges[minimum]:
            minimum = i
        if laser.ranges[i] < corner_dist:
            if verti > abs(laser.ranges[i] * math.cos(i*laser.angle_increment - (PI))):
                corner = i
                corner_dist = laser.ranges[corner]
                projected_corner_dist = corner_dist * math.sin(corner*laser.angle_increment - (PI))
                verti = abs(corner_dist * math.cos(corner*laser.angle_increment - (PI)))

    #verti = abs(corner_dist * math.cos(corner*laser.angle_increment - (PI)))
    print "Projected corner: ", projected_corner_dist
    print "verti: ", verti
    print "corner dist", corner_dist
    #print "corner dist ", corner_dist
    safety_dist = min(laser.ranges[265:275])
    print "Safety_dist", safety_dist
    #verti < 1
    if safety_dist >  projected_corner_dist + wall_dist + 0.1 and verti >= 0.4 and corner > 181 and corner < 260  and corner_dist < back:
        bool_outer_corner = True
        #print "Outer corner"
    future_corner = 0
    future_verti = 0
    future_dist = 0
    future_projected_dist = 4
    projected_temp = 0
    if not bool_outer_corner:
        for i in range(280, 330):
            projected_temp = abs(laser.ranges[i]* math.sin(corner*laser.angle_increment - (1.5*PI)))
            if projected_temp < future_projected_dist:
                future_projected_dist = projected_temp
                future_corner = i
                future_dist = laser.ranges[future_corner]
                future_verti = abs(laser.ranges[i]* math.cos(corner*laser.angle_increment - (1.5*PI)))
        if future_projected_dist > wall_dist + 0.1 and verti >= 0.3 and corner > 181 and corner < 260  and corner_dist < back:
            bool_outer_corner = True


def is_inner_corner(laser):
    """
    Assessment of inner corner in the wall.
    If the three front regions are inferior than the wall_dist.
    If all the elements in last_kinds_of_wall are 'I' and the last time a real corner was detected is superior or equal to 20 seconds:
        To state_outer_inner a 'I' is appended and
        The time is restart.
    Returns:
            bool_inner_corner: 0 if it is not a inner corner; 1 if it is a inner corner
    """

    global bool_inner_corner, regions_, wall_dist, last_kinds_of_wall, last_inner_corner_detection_time, index, state_outer_inner, index_state_outer_inner, loop_index_inner_corner, loop_index
    count = 0
    regions = regions_
    max_distance = 0
    min_dist_front = min(min(laser.ranges[0:10]),min(laser.ranges[358:359]))
    max_dist_front = max(max(laser.ranges[0:10]),max(laser.ranges[358:359]))
    max_distance = max(max(laser.ranges[0:45]),max(laser.ranges[225:359]))
    mini = min(min(laser.ranges[0:30]),min(laser.ranges[330:359]))
    print "Max distance: ", max_distance
    #if max_distance < wall_dist*2:
    #    bool_inner_corner = 1    #print "Inner corner"
    print "min: ", mini
    if min_dist_front < (wall_dist) and laser.ranges[270]<1:
        if max_dist_front < wall_dist *1.5:
            bool_inner_corner = 1
    if  mini < 0.3 :
        bool_inner_corner = 1


def obstacle_detection():
    '''
    function for finding the nearest wall from which it is possible to start
    the laser scan. Basically, from the laser scan this function finds the direction
    and the distance of the nearest obstacle that satisfies some conditions, like
    a certain amount of consecutive cells with a similar distance
    '''
    global dist_front, wall_found
    #print "Obstacle_detection"
    laser = rospy.wait_for_message("/scan",LaserScan)

    if oriented:
        dist_front = laser.ranges[0]

    found = 0
    obstacle_direction = [0 for x in range(len(laser.ranges))]
    points = [0 for x in range(len(laser.ranges))]
    ob_count = 0
    global steps

    steps = len(laser.ranges)
    i = 0

    while (i < steps + laser_points):
        k = 0
        if ((laser.ranges[i%steps] < d) and (laser.ranges[i%steps]>laser.range_min) and (laser.ranges[i%steps] > laser.ranges[(i-1)%steps] - 0.1) and (laser.ranges[i%steps] < laser.ranges[(i-1)%steps] + 0.1)):
            points[ob_count] = points[ob_count] + 1
        elif points[ob_count] > 1:
            obstacle_direction[ob_count] = i%steps;
            ob_count = ob_count + 1
        i = i + 1
    if (ob_count != 0):
        obstacle_distance = [0 for x in range(ob_count)]
        print "Found some obstacles"
        wall_found = 1
        for k in range(ob_count):
            i = 0
            min = d
            s = obstacle_direction[k]
            while  i < points[k]:
                if (laser.ranges[(s-i)%steps] < min) and (laser.ranges[(s-i)%steps] > laser.range_min):
                    min = laser.ranges[(s-i)%steps];
                    obstacle_direction[k] = (s-i)%steps
                    obstacle_distance[k] = laser.ranges[(s-i)%steps]
                i = i + 1
        if (abs(obstacle_direction[0] - obstacle_direction[ob_count-1]) < 10):
            obstacle_direction[0] = obstacle_direction[ob_count-1]
            points[0] = points[ob_count-1]

        nearest_obstacle(obstacle_direction, points, ob_count, obstacle_distance)
    else:
        print "Not obstacles in ", d, " meter"
        wall_found = 0

def nearest_obstacle(direction, points, N, distances):
    '''
    Function for finding the nearest obstacle among the ones that satisfy the previous conditions
    Moreover, it is also defined the message to be sent
    '''
    #print "nearest_obstacle"
    max_cells = points[0]
    distance_nearest = 0
    direction_nearest = 0
    for k in range(N):
        if points[k] >= max_cells:
            max_cells = points[k]
            distance_nearest = distances[k]
            direction_nearest = direction[k]
    print "The nearest obstacle is at direction", direction_nearest

    ob.cells = max_cells
    ob.distance = distance_nearest
    ob.direction = direction_nearest


def approaching():

    #print "Begin to approach the wall"
    global reached, wall_reached, msg, rate, oriented, starting_detection, goal_direction

    if not reached:
        #rate.sleep()
        #print("Waiting for obstacle")
        if starting_detection:
            obstacle_detection()
            starting_detection = False
            goal_direction = ob.direction + current_angle_z()
            if goal_direction  > 360:
                goal_direction = abs(goal_direction) - 360
        if ob.cells > min_cells:
            #print "Possible wall found"
            #print "Calling angular reach function"
            if not oriented:

                angular_reach(goal_direction)
                print "Not oriented"            # Motion to desired direction

            if (not reached) and oriented:
                obstacle_detection()

                if ob.distance >= ref_d:
                    linear_reach(ob.distance)            # Proceeding to the obstacle -- until ref distance
                    # Publish at the desired rate.
                else:
                # Stopping our robot after the movement is over.
                    print("Obstacle reached")
                    msg.linear.x = 0
                    msg.angular.z = 0
                    reached = True
                    wall_reached = True
        else:
            print("NO BIG OBSTACLES")
            wall_reached = False
            goal_direction = current_angle_z()+90
            if goal_direction  > 360:
                goal_direction = abs(goal_direction) - 360

            angular_reach(goal_direction)             # Motion to desired direction

            if not ob.cells > min_cells:

                obstacle_detection()

                linear_reach(2)            # Proceeding to the obstacle -- until ref distance

                # Publish at the desired rate.
                rate.sleep()

            # Stopping our robot after the movement is over.

            print("Obstacle found")
            msg.linear.x = 0
            msg.angular.z = 0



def current_angle_z():

    #print "Current_angle"
    odom = rospy.wait_for_message('odom', Odometry)

    orientation_q = odom.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)


    deg_yaw = yaw*180/PI

    if (deg_yaw<0):
        #print('angle' + str(angle))
        deg_yaw = 360 - abs(deg_yaw)

    # print("Current angle (deg): " + str(deg_yaw))

    return deg_yaw

def angular_reach(data):

    #print "angular_reach"

    global msg, oriented, bool_inner_corner, starting_rotation
    direction = -1
    print "rotation", starting_rotation
    print("Rotating towards: " + str(data))



    msg.angular.z = 0
    msg.linear.x=0
    msg.linear.y=0
    msg.linear.z=0
    msg.angular.x = 0
    msg.angular.y = 0
    current_angle = current_angle_z()



    if not (data-2<current_angle < data+2) and not oriented:
        msg.angular.z = angular_speed*direction
        print "current_angle: ", current_angle
        print "Goal: ", data

        #print("Current angle (deg): " + str(current_angle))
    elif (data-2<current_angle < data+2):
        msg.angular.z = 0
        oriented = True
        print("Oriented!")

    #vel_pub.publish(vel_msg)




def linear_reach(distance):

        print "Linear_reach"
        global msg

        msg.angular.z = 0
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0

        print("Distance: " + str(ob.distance))

        msg.linear.x = 0.2 * (dist_front - ref_d)
        #vel_pub.publish(vel_msg)


def clbk_laser(msg):
    """
    Read sensor messages, and determine distance to each region.
    Manipulates the values measured by the sensor.
    Callback function for the subscription to the published Laser Scan values.
    """
    global regions_, angle_min, dist_front,dist_min, direction, right, back, right_bool, min_index, min_index_right, dist_min_right
    global outside
    size = len(msg.ranges)

    #Define the direction of the nearest obstacle
    for i in range(265, 275):
        if msg.ranges[i] < msg.ranges[min_index_right] and msg.ranges[i] > 0.01:
            min_index_right = i

    #finding the nearest obstacle
    for i in range(180, 300):
        if msg.ranges[i] < msg.ranges[min_index] and msg.ranges[i] > 0.01:

            min_index = i #finding the nearest obstacle

    angle_min = (min_index_right)*msg.angle_increment #angle wrt the nearest obstacle
    #print "Minimum angle ", angle_min
    dist_min = msg.ranges[min_index] #minimum distance
    dist_min_right = msg.ranges[min_index_right]
    #print "Minimum_distance: ", dist_min
    dist_front = msg.ranges[0] #distance from the bot front



    if state_ == 2:
        if max(msg.ranges[255:284]) < 1.5 and max(msg.ranges[265:275]) - min(msg.ranges[265:275]) <= min(msg.ranges[265:275])/2:
            right_bool = 1
        else:
            right_bool = 0


        is_outer_corner(min_index,msg)
        is_inner_corner(msg)

        if outside_check(msg):
            outside = True
    elif state_==3 or state_==4:
        bool_inner_corner = 0
        bool_outer_corner = 0
        right_bool = 0




def following_wall():
    """
    Proportional control for the wall following state.
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> 0; 0.5max_speed; 0.4max_speed
                    msg.angular.z -> PD controller response
    """


    global wall_dist,  dist_min, dist_front, e, angle_min, msg, delta_t, dist_min_right




    #if dist_front >= wall_dist and dist_front != inf:
        #msg.linear.x = 0.1
    #elif dist_front == inf:
        #msg.linear.x = 0.1
    #else:
        #msg.linear.x = 0
    print "Angle minimum: ", angle_min*180/PI
    msg.linear.x = 0
    if right_bool:
        msg.linear.x = 0.1
        msg.angular.z = 2 * (angle_min - (1.5*PI))

    else:
        msg.angular.z = 0
        msg.linear.x = 0.1

    #print "Velocita angolare ", msg.angular.z
    #print "Minimum angle: ", angle_min
    delta_t = rospy.Time.now().to_sec() - t0

def change_state(state):
    """
    Update machine state
    """
    global state_,  state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state
    msg.linear.x = 0
    msg.angular.z = 0

def take_action():
    """
    Change state for the machine states
            State 0 Try to find a wall
            State 1 Wall found - approaching to wall
            State 2 Following wall
    """
    #print "Take action"
    global regions_, oriented, Init, starting_rotation, current_angle,t0, delta_t, outside

    global vel_pub,wall_dist, max_speed, direction, p, d, angle, dist_min, wall_found, rotating, bool_outer_corner, bool_inner_corner

    regions = regions_

    msg.linear.x = 0
    msg.angular.z = 0

    state_description = ''



    if wall_found and not wall_reached and Init:
       change_state(1)
       #print "Changin state to approaching"
       wall_found = False
       Init = False
       starting_rotation = True
       current_angle = 0
    elif state_==1 and wall_reached:
        oriented = False
        print "Starting rotation: ", starting_rotation
        if starting_rotation == True:
            current_angle = current_angle_z()
            starting_rotation = False
        angular_reach((current_angle + 90)%360)
        if oriented:
            change_state(2)
            t0=rospy.Time.now().to_sec()
            sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
            wall_reached == False
    elif state_==2 and outside and delta_t > 7:
        change_state(5)
        delta_t = 0
        oriented = False
        starting_rotation = True
        outside = False
    elif state_== 2 and bool_inner_corner and not bool_outer_corner:
        change_state(3)
        oriented = False
        starting_rotation = True
    elif state_== 2 and bool_outer_corner and delta_t > 7:
        change_state(4)
        delta_t = 0
        oriented = False
        starting_rotation = True
    elif (state_==3 or state_==4) and oriented:
        change_state(2)
        t0=rospy.Time.now().to_sec()
        bool_inner_corner = False
        bool_outer_corner = False
    elif (state_ == 5) and oriented:
        change_state(2)
        t0=rospy.Time.now().to_sec()
        outside = False
    if state_ == 0:
        #print "Finding wall"
        obstacle_detection()
    if state_ == 1:
        #print "Approaching to wall"
        approaching()
    if state_ == 2:
        #print "Following wall"

        following_wall()
    if state_ == 3:
        print "Rotating for inner corner"
        if starting_rotation:
            angle = current_angle_z()
            print "Starting angle", angle
            starting_rotation = False
        angular_reach((angle + 90)%360)
    if state_ == 4:
        print "Rotating for outer corner"
        if starting_rotation:
            angle = current_angle_z()
            if angle > 0 and angle < 90:
                angle = 360 + angle
            print "Starting angle", angle - 360
            starting_rotation = False
        angular_reach((angle - 90)%360)
    if state_ ==5:
        print "Rotating of 180 degrees"
        if starting_rotation:
            angle = current_angle_z()
            if angle > 0 and angle < 90:
                angle = 360 + angle
            print "Starting angle", angle - 360
            starting_rotation = False
        angular_reach((angle + 180)%360)
    vel_pub.publish(msg)


def main():

    global vel_pub, active_, hz, msg, rate

    rospy.init_node('Wall_follower')

    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    msg = Twist()

    #sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    print 'Code is running'
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():

        take_action()

        # State Dispatcher


        rate.sleep()

if __name__ == '__main__':
    main()

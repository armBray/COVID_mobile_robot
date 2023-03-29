#! /usr/bin/env python

#possibili miglioramenti: usare funzioni per trovare min e indice corrispondendte per direzione ostacolo
#risolvere overlap primo ostacolo identificato
#definire ostacoli come classe
import rospy
from sensor_msgs.msg import LaserScan
from obstacle_detection.msg import Obstacle
from geometry_msgs.msg import Twist
import math


#max distance at which our robot can see obstacles
d = 1
#amount of laser point for defining a Wall
laser_points = 60

vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
class obst:
    def __init__(self, cells, direction, distance):
        self.cells = cells
        self.direction = direction
        self.distance = distance
    def update(self, cells, direction, distance):
        self.cells = cells
        self.direction = direction
        self.distance = distance
ob = obst(0,0,10)

def obstacle_detection(msg):

    obstacle_direction = [0 for x in range(len(msg.ranges))]
    points = [0 for x in range(len(msg.ranges))]
    ob_count = 0
    global steps
    steps = len(msg.ranges)
    i = 0
    #print "Before the loop"
    while (i < steps + laser_points):
        #print "ranges[%s] = %s, ranges_min = %s", i, msg.ranges[i], msg.range_min
        k = 0
        if ((msg.ranges[i%steps] < d) and (msg.ranges[i%steps]>msg.range_min) and (msg.ranges[i%steps] > msg.ranges[(i-1)%steps] - 0.1) and (msg.ranges[i%steps] < msg.ranges[(i-1)%steps] + 0.1)):
            points[ob_count] = points[ob_count] + 1
        elif points[ob_count] > 1:
            obstacle_direction[ob_count] = i%steps;
            ob_count = ob_count + 1
        i = i + 1
    if (ob_count != 0):
        obstacle_distance = [0 for x in range(ob_count)]
        print "Found some obstacles"
        for k in range(ob_count):
            i = 0
            min = d
            s = obstacle_direction[k]
            while  i < points[k]:
                if (msg.ranges[(s-i)%steps] < min) and (msg.ranges[(s-i)%steps] > msg.range_min):
                    min = msg.ranges[(s-i)%steps];
                    obstacle_direction[k] = (s-i)%steps
                    obstacle_distance[k] = msg.ranges[(s-i)%steps]
                i = i + 1
        if (abs(obstacle_direction[0] - obstacle_direction[ob_count-1]) < 10):
            obstacle_direction[0] = obstacle_direction[ob_count-1]
            points[0] = points[ob_count-1]
        nearest_obstacle(obstacle_direction, points, ob_count, obstacle_distance)

def nearest_obstacle(direction, points, N, distances):
    max_cells = points[0]
    distance_nearest = 0
    direction_nearest = 0
    for k in range(N):
        if points[k] >= max_cells:
            max_cells = points[k]
            distance_nearest = distances[k]
            direction_nearest = direction[k]
    print "The nearest obstacle is at direction", direction_nearest
    obst.update(ob,max_cells, direction_nearest,distance_nearest)


def communication():

    pub = rospy.Publisher('berto', Obstacle, queue_size=10)
    Ob = Obstacle()
    Ob.distance = ob.distance
    Ob.cells = ob.cells
    Ob.direction = ob.direction
    print "Publishing"
    pub.publish(Ob)
    print "Publishing done"

def fsm():
    global state

    laser = rospy.wait_for_message("/scan",LaserScan)

    obstacle_detection(laser)
    vel = Twist()
    print "Distance from the nearest obstacle: ", ob.distance
    if ob.distance > d:
        print "No obstacles in ",d, "meters"
        state = 0
    else:
        state = 1
        print "There are some obstacles"

    if state == 1:
         vel.linear.x = 0
         vel_pub.publish(vel)
         communication()
    elif state ==0:
         vel.linear.x = 0.08
         t0 = rospy.Time.now().to_sec()
         t1 = 0

         while not (t1 >= 0.1):
             vel_pub.publish(vel)
             t1 = rospy.Time.now().to_sec() - t0
         print "A second is gone"



def main():

    rospy.init_node('Radar')

    #sub = rospy.Subscriber('/scan',LaserScan,obstacle_detection)


    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        rate.sleep()
        fsm()



if __name__ == '__main__':
    main()

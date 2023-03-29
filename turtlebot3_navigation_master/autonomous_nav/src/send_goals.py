#! /usr/bin/env python

import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty


class MoveBaseSeq():

    def __init__(self):
	
        rospy.init_node('move_base_sequence')

        self.current_goal = MoveBaseGoal()
        self.goal_cnt = 0
        self.located = True

        # Localization Sequence - square
        points_loc = [1,1]  #[x1,y1,z1 ... ,xN,yN,zN]    with respect to base link
        angles_loc = [180]

        # Goal Sequence
        points_seq = rospy.get_param('send_goals/p_seq')
        # Only yaw angle required (no ratotions around x and y axes) in deg:
        angles_seq = rospy.get_param('send_goals/yea_seq')
        #List of goal quaternions:
        quat_seq = list()
        #List of goal poses:
        self.pose_loc = list()
        self.pose_seq = list()

        # Definition of Quaternions - localization
        for angle in angles_loc:
            #Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, angle*math.pi/180, axes='sxyz'))))
        n = 2
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_loc[i:i+n] for i in range(0, len(points_loc), n)]

        n = 3
        # Definition of overall sequence of localization Poses
        for point in points:
            #Adding z variable
            point.append(0)
            #Exploit n variable to cycle in quat_seq
            self.pose_loc.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1

        quat_seq = list()

        # Definition of Quaternions - goals
        for angle in angles_seq:
            #Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, angle*math.pi/180, axes='sxyz'))))
        n = 2
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]

        n = 3
        # Definition of overall sequence of goal Poses
        for point in points:
            #Adding z
            point.append(0)
            #Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
	
        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Start localization")

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        if not self.located:
            self.locate()
        else:
            self.movebase_client()

        #self.movebase_client()

    def locate(self):
        # Spread particles
        rospy.wait_for_service('global_localization')
        try:
            global_localization = rospy.ServiceProxy('global_localization', Empty)
            global_localization()
            rospy.loginfo("AMCL Particles spread")
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # Move around to locate

        self.movebase_client()


    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):

        self.goal_cnt += 1
    # Terminal Status from Reference http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")

            if not self.located:

                if self.goal_cnt>=len(self.pose_loc):
                    self.located = True
                    self.goal_cnt = 0
                    rospy.loginfo("Localization terminated - Start Motion")

                self.movebase_client()
            else:
                if self.goal_cnt< len(self.pose_seq):

                    next_goal = MoveBaseGoal()
                    next_goal.target_pose.header.frame_id = "map"
                    next_goal.target_pose.header.stamp = rospy.Time.now()
                    next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                    rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                    rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                    self.client.send_goal(next_goal, self.done_cb, self.active_cb)

                else:

                    rospy.loginfo("Final goal pose reached!")
                    rospy.signal_shutdown("Final goal pose reached!")
                    return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt+1)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt+1)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):

        goal = MoveBaseGoal()
        vel_msg = Twist()

        if not self.located:

            tt = 0
            vel_msg.angular.z = 0
            vel_msg.linear.x=0
            vel_msg.linear.y=0
            vel_msg.linear.z=0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0

            while tt < 5000:
                rospy.loginfo("Rotating")
                vel_msg.angular.z = 0.5
                vel_msg.linear.x = 0
                self.pub.publish(vel_msg)
                tt += 1

            vel_msg.angular.z=0
            vel_msg.linear.x = 0
            tt = 0

            while tt < 5000:
                rospy.loginfo("Proceeding")
                vel_msg.linear.x = 0.1
                self.pub.publish(vel_msg)
                tt += 1

            vel_msg.linear.x = 0

            goal.target_pose.header.frame_id = "base_footprint"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.pose_loc[self.goal_cnt]
            rospy.loginfo("Sending location pose "+str(self.goal_cnt+1)+" to Action Server")
            rospy.loginfo(str(self.pose_loc[self.goal_cnt]))
        else:
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.pose_seq[self.goal_cnt]
            rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
            rospy.loginfo(str(self.pose_seq[self.goal_cnt]))

        self.client.send_goal(goal, self.done_cb, self.active_cb)#, self.feedback_cb)
        rospy.spin()


if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

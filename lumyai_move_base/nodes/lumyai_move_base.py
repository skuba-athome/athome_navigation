#! /usr/bin/python

import roslib
roslib.load_manifest('lumyai_move_base')

import rospy
import actionlib

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped, Point, Quaternion, Twist, Pose2D
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from lumyai_navigation_msgs.msg import NavGoalMsg
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from math import pow, sqrt, cos, sin
from std_msgs.msg import String

class NavState():
    def __init__(self):
        
        rospy.init_node('lumyai_move_base', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)
        
        # Goal state return values 
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED', 'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server") 

        # setup our call move_base/clear_costmaps service 
        rospy.wait_for_service('move_base_node/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('move_base_node/clear_costmaps',Empty)
        
        # Get the initial pose from the user
        #rospy.loginfo("Click on the map in RViz to set the intial pose...")
        #self.initial_pose = rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        #rospy.loginfo("Initial Pose:\n" + str(self.initial_pose.pose.pose))
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

        #wait for navGoal msg
        rospy.Subscriber('base/set_pos', NavGoalMsg, self.nav_Getgoal)
        self.nav_status = rospy.Publisher('base/is_fin', String)
        
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        # Make sure we have the initial pose
        #while self.initial_pose.header.stamp == "":
        #    rospy.sleep(1)
            
        rospy.loginfo("Starting navigation")
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            state = self.move_base.get_state()
            rospy.loginfo(str(goal_states[state]))
            self.nav_status.publish(String(str(goal_states[state])))
            rate.sleep()

    def update_initial_pose(self, initial_pose):
        #self.initial_pose = initial_pose
        self.clear_obstacles(self.clear_costmaps)
        rospy.loginfo("Initial Pose:\n" + str(initial_pose.pose.pose))
                          
    def nav_Getgoal(self,nav_goal):
		self.move_base.cancel_goal()
		
		if nav_goal.text_msg == 'clear':
			self.clear_obstacles(self.clear_costmaps)
            
		#rospy.loginfo("text msg: " + str(nav_goal.text_msg))
		#rospy.loginfo("frame reference: " + str(nav_goal.ref_frame))

		if nav_goal.text_msg == 'people':
			rospy.loginfo("extended goal point")
			goal_offset = 0.8
			new_goal = Pose2D()
			new_goal.x = nav_goal.pose2d.x - goal_offset*cos(nav_goal.pose2d.theta)
			new_goal.y = nav_goal.pose2d.y - goal_offset*sin(nav_goal.pose2d.theta)
			new_goal.theta = nav_goal.pose2d.theta
			nav_goal.pose2d = new_goal
			
		goal_pose = Pose()
		goal_pose.position.x = nav_goal.pose2d.x
		goal_pose.position.y = nav_goal.pose2d.y
		goal_pose.position.z = 0
		
		temp = quaternion_from_euler(0, 0, nav_goal.pose2d.theta)
		goal_pose.orientation.x = temp[0]
		goal_pose.orientation.y = temp[1]
		goal_pose.orientation.z = temp[2]
		goal_pose.orientation.w = temp[3]

		goal = MoveBaseGoal()
		goal.target_pose.pose = goal_pose

		if nav_goal.ref_frame == 'relative':
			goal.target_pose.header.frame_id = 'base_link'
		else:
			goal.target_pose.header.frame_id = 'map'
		
		goal.target_pose.header.stamp = rospy.Time.now()

		#rospy.loginfo("Going to:\n" + str(goal_pose)) 
		# Start the robot toward the goal
		self.move_base.send_goal(goal)

    def clear_obstacles(self,clear_costmaps):
        try:
            rospy.loginfo("Cleaning costmaps...")
            clear_costmaps()
        except rospy.ServiceException, e:
            rospy.loginfo("Service did not process request")

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
      
if __name__ == '__main__':
    try:
        NavState()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

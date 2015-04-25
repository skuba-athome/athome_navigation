#! /usr/bin/python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def done_cab(status, result):
        print "DOne"
	print status, result

def active_cab():
        print "active"

def fed_cab(feedback):
        print "feedback"
        print feedback

if __name__ == '__main__':
    rospy.init_node('test_move_base_action')

    client = actionlib.SimpleActionClient('/navigation/move_base', MoveBaseAction)
    client.wait_for_server()

    rospy.loginfo('Initial complete')

    # set goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'base_link'
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.orientation.w = 1.0
    client.send_goal(goal, done_cb=done_cab, active_cb=active_cab, feedback_cb=fed_cab)

    rospy.spin()
    



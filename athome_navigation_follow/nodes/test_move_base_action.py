#! /usr/bin/python

import rospy
import actionlib
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback, MoveBaseActionResult
from std_msgs.msg import String


def done_cab(status, result):
        print "DOne"
	print status, result

def active_cab():
        print "active"

def fed_cab(feedback):
        print "feedback"
        print feedback

def cb_status(data):
        print 'cb_status'
	print data.status.status

def cb_feedback(data):
        position = data.feedback.base_position.pose.position
        orientation = data.feedback.base_position.pose.orientation
        quaternion = (0, 0, orientation.z, orientation.w)
        rpy_angle = euler_from_quaternion(quaternion)
        b_position = (position.x, position.y, rpy_angle[2])
        print 'cb_feedback'
	print b_position

if __name__ == '__main__':
    rospy.init_node('test_move_base_action')

    rospy.Subscriber('/navigation/move_base/result', MoveBaseActionResult, cb_status)
    rospy.Subscriber('/navigation/move_base/feedback', MoveBaseActionFeedback, cb_feedback)

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
    



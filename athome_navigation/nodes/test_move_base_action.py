#! /usr/bin/python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction


if __name__ == '__main__':
    rospy.init_node('test_move_base_action')

    client = actionlib.SimpleActionClient('/navigation/move_base_node', MoveBaseAction)
    client.wait_for_server()

    rospy.loginfo('Initial complete')
    # set goal

